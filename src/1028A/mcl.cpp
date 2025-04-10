#include "1028A/mcl.h"
#include "1028A/robot.h"
#include <random>
#include <cmath>
#include <algorithm>
#include <iostream>

namespace _1028A {

Particle::Particle(double xx, double yy, double tt, double ww)
  : x(xx), y(yy), theta(tt), weight(ww) {
}

// ------------------- HELPER FUNCTIONS --------------------

namespace {
  // Returns uniform random in [0, 1)
  double randomDouble01() {
    static thread_local std::random_device rd;
    static thread_local std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    return dist(gen);
  }

  // Returns a Gaussian random number with the given mean/stdev
  double gaussianRandom(double mean, double stddev) {
    static thread_local std::random_device rd;
    static thread_local std::mt19937 gen(rd());
    std::normal_distribution<double> dist(mean, stddev);
    return dist(gen);
  }

  // Probability density of x under a Gaussian(mean, stddev)
  double gaussProb(double mean, double stddev, double x) {
    double d = x - mean;
    double exponent = -(d * d) / (2.0 * stddev * stddev);
    double coeff = 1.0 / (std::sqrt(2.0 * M_PI) * stddev);
    return coeff * std::exp(exponent);
  }
}

// ------------------- CLASS IMPLEMENTATION --------------------

/**
 * Constructor:
 *  - Creates distance sensors
 *  - Stores odom function pointers
 *  - fieldX, fieldY => total field size in inches (12ft => 144).
 *    So boundaries are +/- fieldX/2, +/- fieldY/2 around (0,0).
 *  - Calls initParticles to randomly place them in [-72, +72], [-72, +72].
 *  - Forces lastX,Y,Theta = 0 => MCL starts from (0,0,0).
 */
MonteCarloLocalizer::MonteCarloLocalizer(
    int frontPort,
    int backPort,
    int leftPort,
    int rightPort,
    std::function<double()> odomXFun,
    std::function<double()> odomYFun,
    std::function<double()> odomThetaFun,
    double fieldXIn,
    double fieldYIn,
    int    numParticles)
  : frontDist(frontPort),
    backDist(backPort),
    leftDist(leftPort),
    rightDist(rightPort),
    odomX(odomXFun),
    odomY(odomYFun),
    odomTheta(odomThetaFun),
    fieldX(fieldXIn),
    fieldY(fieldYIn),
    lastX(0.0),
    lastY(0.0),
    lastTheta(0.0)
{
  // 1) Initialize the particles.
  initParticles(numParticles);

  // 2) Force the internal reference to (0,0,0)
  lastX = 0.0;
  lastY = 0.0;
  lastTheta = 0.0;
}

/**
 * Single-step MCL:
 *  - Read current odom, compute dx/dy/dTheta
 *  - motionUpdate (with noise)
 *  - sensorUpdate (distances)
 *  - resample
 *  - getBest
 *  - update lastX/Y/Theta
 */
Particle MonteCarloLocalizer::updateLocalization(double transNoise,
                                                double rotNoise,
                                                double sensorNoise)
{
  // 1) Current odom from your system
  double currX = odomX();      // might be 33,57 if your system says that
  double currY = odomY();
  double currTheta = odomTheta();

  // 2) Delta from last update
  double dx = currX - lastX;
  double dy = currY - lastY;
  double dTheta = angleDiff(currTheta, lastTheta);

  // 3) Motion update
  motionUpdate(dx, dy, dTheta, transNoise, rotNoise);

  // 4) Read sensors in mm => convert to inches
  double frontVal = mmToInches(frontDist.get());
  double backVal  = mmToInches(backDist.get());
  double leftVal  = mmToInches(leftDist.get());
  double rightVal = mmToInches(rightDist.get());

  // If negative => sensor error => clamp to 0
  if(frontVal < 0) frontVal = 0;
  if(backVal  < 0) backVal  = 0;
  if(leftVal  < 0) leftVal  = 0;
  if(rightVal < 0) rightVal = 0;

  // 5) Sensor update
  sensorUpdate(frontVal, backVal, leftVal, rightVal, sensorNoise);

  // 6) Resample
  resample();

  // 7) If all weights are near zero, re-init to not get stuck
  reinitIfWeightsZero();

  // 8) Best particle
  Particle best = getBestParticle();

  // 9) Update last odom so next time we compute new delta
  lastX = currX;
  lastY = currY;
  lastTheta = currTheta;

  return best;
}

/**
 * Static function: runs in a background task for repeated MCL updates.
 */
void MonteCarloLocalizer::mclTaskFn(void* param) {
  auto* mcl = static_cast<MonteCarloLocalizer*>(param);
  while (true) {
    mcl->updateLocalization(mcl->taskTransNoise,
                            mcl->taskRotNoise,
                            mcl->taskSensorNoise);
    pros::delay(mcl->taskDelay);
  }
}

/**
 * Creates a new PROS task that does repeated MCL updates.
 */
void MonteCarloLocalizer::startTask(double transNoise,
                                    double rotNoise,
                                    double sensorNoise,
                                    uint32_t taskDelayMs)
{
  taskTransNoise  = transNoise;
  taskRotNoise    = rotNoise;
  taskSensorNoise = sensorNoise;
  taskDelay       = taskDelayMs;

  if (mclTask != nullptr) {
    mclTask->remove();
    delete mclTask;
    mclTask = nullptr;
  }
  mclTask = new pros::Task(mclTaskFn, this, "MCL_Task");
}

// ----------------------- Private methods -----------------------

/**
 * Particles placed randomly:
 *   x in [-fieldX/2, +fieldX/2]
 *   y in [-fieldY/2, +fieldY/2]
 *   theta in [0, 2π)
 */
void MonteCarloLocalizer::initParticles(int n) {
  particles_.clear();
  particles_.reserve(n);

  for(int i=0; i<n; i++){
    double rx = (randomDouble01() - 0.5) * fieldX; // => [-fieldX/2, +fieldX/2]
    double ry = (randomDouble01() - 0.5) * fieldY; // => [-fieldY/2, +fieldY/2]
    double rt = randomDouble01() * 2.0 * M_PI;      // => [0, 2π)
    particles_.emplace_back(rx, ry, rt, 1.0);
  }
}

/**
 * Move each particle by (dx, dy, dTheta) + noise, 
 * then clamp x,y within boundaries, 
 * then wrap theta in [0, 2π).
 */
void MonteCarloLocalizer::motionUpdate(double dx, double dy, double dTheta,
                                       double transNoise, double rotNoise)
{
  double halfX = fieldX / 2.0;
  double halfY = fieldY / 2.0;

  for(auto &p : particles_) {
    // Add nominal motion + noise
    p.x     += dx + gaussianRandom(0.0, transNoise);
    p.y     += dy + gaussianRandom(0.0, transNoise);
    p.theta += dTheta + gaussianRandom(0.0, rotNoise);

    // Clamp within [-halfX, +halfX] & [-halfY, +halfY]
    if(p.x < -halfX) p.x = -halfX;
    if(p.x >  halfX) p.x =  halfX;
    if(p.y < -halfY) p.y = -halfY;
    if(p.y >  halfY) p.y =  halfY;

    // Wrap angle [0..2π)
    p.theta = std::fmod(p.theta, 2.0*M_PI);
    if(p.theta < 0) {
      p.theta += 2.0*M_PI;
    }
  }
}

/**
 * Compare measured sensor distances (front, back, left, right)
 * with expected distances at each particle.
 * Multiply probabilities => p.weight.
 * Then find max weight to normalize.
 */
void MonteCarloLocalizer::sensorUpdate(double frontMeas, double backMeas,
                                       double leftMeas, double rightMeas,
                                       double sensorNoise)
{
  double maxW = 0.0;
  for(auto &p : particles_) {
    double frontExp = expectedDistance(p.x, p.y, p.theta);
    double backExp  = expectedDistance(p.x, p.y, p.theta + M_PI);
    double leftExp  = expectedDistance(p.x, p.y, p.theta - M_PI/2.0);
    double rightExp = expectedDistance(p.x, p.y, p.theta + M_PI/2.0);

    double w1 = gaussProb(frontExp, sensorNoise, frontMeas);
    double w2 = gaussProb(backExp,  sensorNoise, backMeas);
    double w3 = gaussProb(leftExp,  sensorNoise, leftMeas);
    double w4 = gaussProb(rightExp, sensorNoise, rightMeas);

    p.weight = w1 * w2 * w3 * w4; // multiply for final
    if(p.weight > maxW) {
      maxW = p.weight;
    }
  }

  // Normalize
  if(maxW > 1e-15) {
    for(auto &p : particles_) {
      p.weight /= maxW;
    }
  }
}

/**
 * Resample using the "roulette wheel" approach:
 *   - index = random
 *   - spin beta up to 2*maxWeight
 *   - move index forward until you find the selected particle
 */
void MonteCarloLocalizer::resample() {
  std::vector<Particle> newSet = particles_;

  // Find max weight
  double mw = 0.0;
  for(const auto &p : particles_) {
    if(p.weight > mw) {
      mw = p.weight;
    }
  }

  // If all near zero, we handle that after resample with reinitIfWeightsZero().
  if(mw < 1e-15) {
    return;
  }

  int index = static_cast<int>(randomDouble01() * particles_.size());
  double beta = 0.0;

  for(size_t i=0; i < particles_.size(); i++) {
    beta += randomDouble01() * 2.0 * mw;
    while(beta > particles_[index].weight) {
      beta -= particles_[index].weight;
      index = (index + 1) % particles_.size();
    }
    newSet[i] = particles_[index];
  }
  particles_ = newSet;
}

/**
 * If the max weight after sensor update is near zero,
 * reinitialize all particles to avoid being stuck at zero.
 */
void MonteCarloLocalizer::reinitIfWeightsZero() {
  double mw = 0.0;
  for(const auto &p : particles_) {
    if(p.weight > mw) {
      mw = p.weight;
    }
  }
  // If still < 1e-15 => all effectively zero => reinit
  if(mw < 1e-15) {
    //std::cout << "[MCL] All weights near zero => reinitializing particles.\n";
    initParticles(particles_.size());
  }
}

/**
 * Return highest-weight particle
 */
Particle MonteCarloLocalizer::getBestParticle() const {
  Particle best = particles_.front();
  for(const auto &p : particles_) {
    if(p.weight > best.weight) {
      best = p;
    }
  }
  return best;
}

/**
 * Convert mm to inches. Negative mm => sensor error => keep negative, then clamp above.
 */
double MonteCarloLocalizer::mmToInches(double mm) const {
  static constexpr double MM_PER_INCH = 25.4;
  if(mm < 0) return mm; // indicates sensor error
  return mm / MM_PER_INCH;
}

/**
 * Return smallest signed difference in angles, in [-π, π].
 */
double MonteCarloLocalizer::angleDiff(double a, double b) {
  double diff = a - b;
  while(diff >  M_PI) diff -= 2.0*M_PI;
  while(diff < -M_PI) diff += 2.0*M_PI;
  return diff;
}

/**
 * Find distance from (x,y) to the boundary in "heading". The field is centered at (0,0):
 *  x in [-fieldX/2, +fieldX/2]
 *  y in [-fieldY/2, +fieldY/2]
 * We'll do a ray cast to each boundary plane and pick the smallest positive t.
 */
double MonteCarloLocalizer::expectedDistance(double x, double y, double heading) const {
  double dir = std::fmod(heading, 2.0*M_PI);
  if(dir < 0) dir += 2.0*M_PI;

  double vx = std::cos(dir);
  double vy = std::sin(dir);

  double halfX = fieldX / 2.0;
  double halfY = fieldY / 2.0;

  double tList[4] = {1e9, 1e9, 1e9, 1e9};

  // left boundary:   x = -halfX
  if(vx < -1e-12) {
    tList[0] = ( -halfX - x ) / vx;
  }
  // right boundary:  x = +halfX
  if(vx > 1e-12) {
    tList[1] = ( halfX - x ) / vx;
  }
  // bottom boundary: y = -halfY
  if(vy < -1e-12) {
    tList[2] = ( -halfY - y ) / vy;
  }
  // top boundary:    y = +halfY
  if(vy > 1e-12) {
    tList[3] = ( halfY - y ) / vy;
  }

  double minT = 1e9;
  for(int i=0; i<4; i++){
    if(tList[i] > 0 && tList[i] < minT) {
      minT = tList[i];
    }
  }
  return minT;
}

// ------------------- Example Odom Getters -------------------
double getX() {
  // E.g., if your code tracks the robot at (33,57), that might appear here:
  return _1028A::robot::chassis.getPose().x;
}
double getY() {
  return _1028A::robot::chassis.getPose().y;
}
double getTheta() {
  return _1028A::robot::chassis.getPose().theta * (M_PI / 180.0);
}

} // namespace _1028A