#pragma once

#include "main.h"
#include <vector>
#include <functional>

namespace _1028A {

/**
 * \brief A single MCL particle: (x, y, theta, weight).
 * - x,y in **inches**
 * - theta in **radians**
 * - weight is the particle's importance
 */
struct Particle {
    double x;
    double y;
    double theta;
    double weight;

    Particle(double xx=0.0, double yy=0.0, double tt=0.0, double ww=1.0);
};

/**
 * \class MonteCarloLocalizer
 * \brief Performs Monte Carlo Localization on a rectangular field using
 * four distance sensors (front, back, left, right) and external odometry.
 *
 * - (0,0) is the center of the field.
 * - Boundaries at +/- (fieldX/2), +/- (fieldY/2).
 * - No wrapping; positions are clamped to field boundaries.
 * - The system starts at (0,0,0) internally.
 */
class MonteCarloLocalizer {
  public:
    /**
     * \brief Constructor for MonteCarloLocalizer
     *
     * @param frontPort     Port # for front distance sensor
     * @param backPort      Port # for back distance sensor
     * @param leftPort      Port # for left distance sensor
     * @param rightPort     Port # for right distance sensor
     * @param odomXFun      Function returning odom X (inches)
     * @param odomYFun      Function returning odom Y (inches)
     * @param odomThetaFun  Function returning odom Theta (radians)
     * @param fieldXIn      Field size in X dimension (inches)
     * @param fieldYIn      Field size in Y dimension (inches)
     * @param numParticles  Number of MCL particles to initialize
     */
    MonteCarloLocalizer(int frontPort, int backPort, int leftPort, int rightPort,
                        std::function<double()> odomXFun,
                        std::function<double()> odomYFun,
                        std::function<double()> odomThetaFun,
                        double fieldXIn=144.0,
                        double fieldYIn=144.0,
                        int    numParticles=300);

    /**
     * \brief Perform one MCL update cycle (blocking call):
     *  1) Motion update (based on odometry delta)
     *  2) Sensor update (distance sensors)
     *  3) Resample
     *  4) Return best (highest-weight) Particle
     *
     * @param transNoise   translation noise (inches stdev)
     * @param rotNoise     rotation noise (radians stdev)
     * @param sensorNoise  distance-sensor noise (inches stdev)
     */
    Particle updateLocalization(double transNoise=0.2,
                                double rotNoise=0.02,
                                double sensorNoise=3.0);

    /**
     * \brief Start a background task that periodically calls updateLocalization().
     * @param transNoise   translation noise (inches stdev)
     * @param rotNoise     rotation noise (radians stdev)
     * @param sensorNoise  distance-sensor noise (inches stdev)
     * @param taskDelayMs  delay in milliseconds between each localization update
     */
    void startTask(double transNoise=0.2,
                   double rotNoise=0.02,
                   double sensorNoise=3.0,
                   uint32_t taskDelayMs=20);

  private:
    // Distance sensors
    pros::Distance frontDist;
    pros::Distance backDist;
    pros::Distance leftDist;
    pros::Distance rightDist;

    // Odometry function pointers (inches, radians)
    std::function<double()> odomX;
    std::function<double()> odomY;
    std::function<double()> odomTheta;

    // Field dimensions (inches)
    // (0,0) is the center => boundaries at +/- (fieldX/2) and +/- (fieldY/2).
    double fieldX;
    double fieldY;

    // Particle set
    std::vector<Particle> particles_;

    // Last odom readings
    double lastX;
    double lastY;
    double lastTheta;

    // -- MCL internals --
    void initParticles(int n);
    void motionUpdate(double dx, double dy, double dTheta,
                      double transNoise, double rotNoise);
    void sensorUpdate(double frontMeas, double backMeas,
                      double leftMeas, double rightMeas,
                      double sensorNoise);
    void resample();
    Particle getBestParticle() const;
    double mmToInches(double mm) const;
    double angleDiff(double a, double b);
    double expectedDistance(double x, double y, double heading) const;

    // If all particles have near-zero weight, re-initialize to avoid being stuck
    void reinitIfWeightsZero();

    // -- Task related --
    static void mclTaskFn(void* param);  // The static task function
    pros::Task* mclTask = nullptr;       // The task pointer

    double taskTransNoise  = 0.2;
    double taskRotNoise    = 0.02;
    double taskSensorNoise = 3.0;
    uint32_t taskDelay     = 20;  // ms
};

// Example odometry getter stubs (adjust for your own system).
double getX();
double getY();
double getTheta();

} // namespace _1028A