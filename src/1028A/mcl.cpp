#include "1028A/mcl.h"
#include "1028A/robot.h"
#include <random>
#include <cmath>
#include <algorithm>
#include <iostream>

namespace _1028A{
    Particle::Particle(double xx, double yy, double tt, double ww)
    : x(xx), y(yy), theta(tt), weight(ww) {
    }

    namespace {
    double randomDouble01() {
        static thread_local std::random_device rd;
        static thread_local std::mt19937 gen(rd());
        std::uniform_real_distribution<double> dist(0.0, 1.0);
        return dist(gen);
    }

    double gaussianRandom(double mean, double stddev) {
        static thread_local std::random_device rd;
        static thread_local std::mt19937 gen(rd());
        std::normal_distribution<double> dist(mean, stddev);
        return dist(gen);
    }

    double gaussProb(double mean, double stddev, double x) {
        double d = x - mean;
        double exponent = -(d * d) / (2.0 * stddev * stddev);
        double coeff = 1.0 / (std::sqrt(2.0*M_PI) * stddev);
        return coeff * std::exp(exponent);
    }
    }

    
    MonteCarloLocalizer::MonteCarloLocalizer(
        int frontPort, int backPort, int leftPort, int rightPort,
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
    // Create particles
    initParticles(numParticles);

    // Capture initial odom
    lastX = odomX();
    lastY = odomY();
    lastTheta = odomTheta();
    }

   
    Particle MonteCarloLocalizer::updateLocalization(double transNoise,
                                                    double rotNoise,
                                                    double sensorNoise)
    {
    // 1) Current odom
    double currX = odomX();
    double currY = odomY();
    double currTheta = odomTheta();

    // 2) Deltas
    double dx = currX - lastX;
    double dy = currY - lastY;
    double dTheta = angleDiff(currTheta, lastTheta);

    // 3) Motion update
    motionUpdate(dx, dy, dTheta, transNoise, rotNoise);

    // 4) Read sensors in mm -> convert to inches
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

    // 7) Get best
    Particle best = getBestParticle();

    // 8) Update last odom
    lastX = currX;
    lastY = currY;
    lastTheta = currTheta;

    return best;
    }

    // ---------------------------------
    // 5) Private methods
    // ---------------------------------

    void MonteCarloLocalizer::initParticles(int n) {
    particles_.clear();
    particles_.reserve(n);
    for(int i=0; i<n; i++){
        double rx = randomDouble01() * fieldX;
        double ry = randomDouble01() * fieldY;
        double rt = randomDouble01() * 2.0*M_PI;
        particles_.emplace_back(rx, ry, rt, 1.0);
    }
    }

    void MonteCarloLocalizer::motionUpdate(double dx, double dy, double dTheta,
                                        double transNoise, double rotNoise)
    {
    for(auto &p : particles_) {
        // Add nominal motion + noise
        p.x     += dx + gaussianRandom(0.0, transNoise);
        p.y     += dy + gaussianRandom(0.0, transNoise);
        p.theta += dTheta + gaussianRandom(0.0, rotNoise);

        // Wrap field
        p.x = std::fmod(std::fmod(p.x, fieldX) + fieldX, fieldX);
        p.y = std::fmod(std::fmod(p.y, fieldY) + fieldY, fieldY);

        // Wrap angle [0..2*pi)
        p.theta = std::fmod(p.theta, 2.0*M_PI);
        if(p.theta < 0) {
        p.theta += 2.0*M_PI;
        }
    }
    }

    void MonteCarloLocalizer::sensorUpdate(double frontMeas, double backMeas,
                                        double leftMeas, double rightMeas,
                                        double sensorNoise)
    {
    double maxW = 0.0;
    for(auto &p : particles_) {
        double frontExp = expectedDistance(p.x, p.y, p.theta + 0.0);
        double backExp  = expectedDistance(p.x, p.y, p.theta + M_PI);
        double leftExp  = expectedDistance(p.x, p.y, p.theta - M_PI/2.0);
        double rightExp = expectedDistance(p.x, p.y, p.theta + M_PI/2.0);

        double w1 = gaussProb(frontExp, sensorNoise, frontMeas);
        double w2 = gaussProb(backExp,  sensorNoise, backMeas);
        double w3 = gaussProb(leftExp,  sensorNoise, leftMeas);
        double w4 = gaussProb(rightExp, sensorNoise, rightMeas);

        p.weight = w1 * w2 * w3 * w4;
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

    void MonteCarloLocalizer::resample() {
    std::vector<Particle> newSet = particles_;

    // Find max weight
    double mw = 0.0;
    for(const auto &p : particles_) {
        if(p.weight > mw) {
        mw = p.weight;
        }
    }
    if(mw < 1e-15) {
        // all near zero => you could re-init or skip
        return;
    }

    int index = (int)(randomDouble01() * particles_.size());
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

    Particle MonteCarloLocalizer::getBestParticle() const {
    Particle best = particles_.front();
    for(const auto &p : particles_) {
        if(p.weight > best.weight) {
        best = p;
        }
    }
    return best;
    }

    double MonteCarloLocalizer::mmToInches(double mm) const {
    static constexpr double MM_PER_INCH = 25.4;
    if(mm < 0) return mm; // indicates sensor error
    return mm / MM_PER_INCH;
    }

    double MonteCarloLocalizer::angleDiff(double a, double b) {
    double diff = a - b;
    while(diff >  M_PI) diff -= 2.0*M_PI;
    while(diff < -M_PI) diff += 2.0*M_PI;
    return diff;
    }

    double MonteCarloLocalizer::expectedDistance(double x, double y, double heading) const {
    double dir = std::fmod(heading, 2.0*M_PI);
    if(dir < 0) dir += 2.0*M_PI;

    double vx = std::cos(dir);
    double vy = std::sin(dir);

    double tList[4] = {1e9,1e9,1e9,1e9};

    // left boundary: x=0
    if(vx < -1e-12) {
        tList[0] = -x / vx;
    }
    // right boundary: x=fieldX
    if(vx > 1e-12) {
        tList[1] = (fieldX - x) / vx;
    }
    // bottom boundary: y=0
    if(vy < -1e-12) {
        tList[2] = -y / vy;
    }
    // top boundary: y=fieldY
    if(vy > 1e-12) {
        tList[3] = (fieldY - y) / vy;
    }

    double minT = 1e9;
    for(int i=0; i<4; i++){
        if(tList[i] > 0 && tList[i] < minT) {
        minT = tList[i];
        }
    }
    return minT; // inches
    }


double getX() {
    return _1028A::robot::chassis.getPose().x; 
}
double getY() {
    return _1028A::robot::chassis.getPose().y; 
}
double getOdom() {
    return _1028A::robot::chassis.getPose().theta * M_PI / 180.0;
  }
}