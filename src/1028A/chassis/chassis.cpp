
#include "1028A/chassis/chassis.hpp"
#include "1028A/chassis/odom.hpp"
#include "1028A/chassis/trackingWheel.hpp"
#include "1028A/pid.hpp"
#include "1028A/util.hpp"
#include "main.h"
#include "pros/misc.hpp"
#include "pros/motors.hpp"
#include <math.h>

_1028A::Chassis::Chassis(Drivetrain_t drivetrain,
                         ChassisController_t lateralSettings,
                         ChassisController_t angularSettings,
                         OdomSensors_t sensors,
                         DriveCurveFunction_t driveCurve) {
  this->drivetrain = drivetrain;
  this->lateralSettings = lateralSettings;
  this->angularSettings = angularSettings;
  this->odomSensors = sensors;
  this->driveCurve = driveCurve;
}

void _1028A::Chassis::calibrate() {
  // calibrate the imu if it exists
  if (odomSensors.imu != nullptr) {
    odomSensors.imu->reset(true);
    // keep on calibrating until it calibrates successfully
    while (errno == PROS_ERR || errno == ENODEV || errno == ENXIO) {
      pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, "---");
      odomSensors.imu->reset(true);
      pros::delay(10);
    }
  }
  // initialize odom
  if (odomSensors.vertical1 == nullptr)
    odomSensors.vertical1 = new _1028A::TrackingWheel(
        drivetrain.leftMotors, drivetrain.wheelDiameter,
        -(drivetrain.trackWidth / 2), drivetrain.rpm);
  if (odomSensors.vertical2 == nullptr)
    odomSensors.vertical2 = new _1028A::TrackingWheel(
        drivetrain.rightMotors, drivetrain.wheelDiameter,
        drivetrain.trackWidth / 2, drivetrain.rpm);
  odomSensors.vertical1->reset();
  odomSensors.vertical2->reset();
  if (odomSensors.horizontal1 != nullptr)
    odomSensors.horizontal1->reset();
  if (odomSensors.horizontal2 != nullptr)
    odomSensors.horizontal2->reset();
  _1028A::setSensors(odomSensors, drivetrain);
  _1028A::init();
  // rumble to controller to indicate success
  pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, ".");
}

void _1028A::Chassis::setPose(float x, float y, float theta, bool radians) {
  _1028A::setPose(_1028A::Pose(x, y, theta), radians);
}

void _1028A::Chassis::setPose(Pose pose, bool radians) {
  _1028A::setPose(pose, radians);
}

_1028A::Pose _1028A::Chassis::getPose(bool radians) {
  return _1028A::getPose(radians);
}

_1028A::Pose _1028A::Chassis::getSpeed(bool radians) {
  return _1028A::getSpeed(radians);
}

_1028A::Pose _1028A::Chassis::getLocalSpeed(bool radians) {
  return _1028A::getLocalSpeed(radians);
}

_1028A::Pose _1028A::Chassis::estimatePose(float time, bool radians) {
  return _1028A::estimatePose(time, radians);
}

void _1028A::Chassis::waitUntilDist(float dist) {
  // do while to give the thread time to start
  do
    pros::delay(10);
  while (distTravelled <= dist && distTravelled != -1);
}

void _1028A::Chassis::turnTo(float x, float y, int timeout, bool async,
                             bool reversed, float maxSpeed, bool log) {
  // try to take the mutex
  // if its unsuccessful after 10ms, return
  if (!mutex.take(10))
    return;
  // if the function is async, run it in a new task
  if (async) {
    pros::Task task(
        [&]() { turnTo(x, y, timeout, false, reversed, maxSpeed, log); });
    mutex.give();
    pros::delay(10); // delay to give the task time to start
    return;
  }
  float targetTheta;
  float deltaX, deltaY, deltaTheta;
  float motorPower;
  float startTheta = getPose().theta;
  std::uint8_t compState = pros::competition::get_status();
  distTravelled = 0;

  // create a new PID controller
  FAPID pid =
      FAPID(0, 0, angularSettings.kP, 0, angularSettings.kD, "angularPID");
  pid.setExit(angularSettings.largeError, angularSettings.smallError,
              angularSettings.largeErrorTimeout,
              angularSettings.smallErrorTimeout, timeout);

  // main loop
  while (pros::competition::get_status() == compState && !pid.settled()) {
    // update variables
    Pose pose = getPose();
    pose.theta =
        (reversed) ? fmod(pose.theta - 180, 360) : fmod(pose.theta, 360);

    // update completion vars
    distTravelled = fabs(angleError(pose.theta, startTheta));

    deltaX = x - pose.x;
    deltaY = y - pose.y;
    targetTheta = fmod(radToDeg(M_PI_2 - atan2(deltaY, deltaX)), 360);

    // calculate deltaTheta
    deltaTheta = angleError(targetTheta, pose.theta);

    // calculate the speed
    motorPower = pid.update(0, deltaTheta, log);

    // cap the speed
    if (motorPower > maxSpeed)
      motorPower = maxSpeed;
    else if (motorPower < -maxSpeed)
      motorPower = -maxSpeed;

    // move the drivetrain
    drivetrain.leftMotors->move(-motorPower);
    drivetrain.rightMotors->move(motorPower);

    pros::delay(10);
  }

  // stop the drivetrain
  drivetrain.leftMotors->move(0);
  drivetrain.rightMotors->move(0);
  // set distTraveled to -1 to indicate that the function has finished
  distTravelled = -1;
  // give the mutex back
  mutex.give();
}

void _1028A::Chassis::moveTo(float x, float y, float theta, int timeout,
                             bool async, bool forwards, float chasePower,
                             float lead, float maxSpeed, bool log) {
  // try to take the mutex
  // if its unsuccessful after 10ms, return
  if (!mutex.take(10))
    return;
  // if the function is async, run it in a new task
  if (async) {
    pros::Task task([&]() {
      moveTo(x, y, theta, timeout, false, forwards, chasePower, lead, maxSpeed,
             log);
    });
    mutex.give();
    pros::delay(10); // delay to give the task time to start
    return;
  }

  Pose target(x, y, M_PI_2 - degToRad(theta)); // target pose in standard form
  Pose lastPose = getPose();                   // last pose
  FAPID linearPID =
      FAPID(0, 0, lateralSettings.kP, 0, lateralSettings.kD, "linearPID");
  FAPID angularPID =
      FAPID(0, 0, angularSettings.kP, 0, angularSettings.kD, "angularPID");
  linearPID.setExit(lateralSettings.largeError, lateralSettings.smallError,
                    lateralSettings.smallErrorTimeout,
                    lateralSettings.smallErrorTimeout,
                    timeout); // exit conditions
  int compState = pros::competition::get_status();
  int start = pros::millis();
  distTravelled = 0;

  if (!forwards)
    target.theta = fmod(target.theta + M_PI, 2 * M_PI); // backwards movement

  bool close = false; // used for settling
  if (chasePower == 0)
    chasePower =
        drivetrain.chasePower; // use global chase power if chase power is 0

  // main loop
  while (pros::competition::get_status() == compState &&
         (!linearPID.settled() || pros::millis() - start < 300)) {
    // get current pose
    Pose pose = getPose(true);
    if (!forwards)
      pose.theta += M_PI;
    pose.theta = M_PI_2 - pose.theta; // convert to standard form

    // update completion vars
    distTravelled += pose.distance(lastPose);
    lastPose = pose;

    // check if the robot is close enough to the target to start settling
    if (pose.distance(target) < 7.5)
      close = true;

    // calculate the carrot point
    Pose carrot = target - (Pose(cos(target.theta), sin(target.theta)) * lead *
                            pose.distance(target));
    if (close)
      carrot = target; // settling behavior

    // calculate error
    float angularError =
        angleError(pose.angle(carrot), pose.theta, true); // angular error
    float linearError =
        pose.distance(carrot) * cos(angularError); // linear error
    if (close)
      angularError =
          angleError(target.theta, pose.theta, true); // settling behavior
    if (!forwards)
      linearError = -linearError;

    // get PID outputs
    float angularPower = -angularPID.update(radToDeg(angularError), 0, log);
    float linearPower = linearPID.update(linearError, 0, log);

    // calculate radius of turn
    float curvature = fabs(getCurvature(pose, carrot));
    if (curvature == 0)
      curvature = -1;
    float radius = 1 / curvature;

    // calculate the maximum speed at which the robot can turn
    // using the formula v = sqrt( u * r * g )
    if (radius != -1) {
      float maxTurnSpeed = sqrt(chasePower * radius * 9.8);
      // the new linear power is the minimum of the linear power and the max
      // turn speed
      if (linearPower > maxTurnSpeed && !close)
        linearPower = maxTurnSpeed;
      else if (linearPower < -maxTurnSpeed && !close)
        linearPower = -maxTurnSpeed;
    }

    // prioritize turning over moving
    float overturn = fabs(angularPower) + fabs(linearPower) - maxSpeed;
    if (overturn > 0)
      linearPower -= linearPower > 0 ? overturn : -overturn;

    // calculate motor powers
    float leftPower = linearPower + angularPower;
    float rightPower = linearPower - angularPower;

    // move the motors
    drivetrain.leftMotors->move(leftPower);
    drivetrain.rightMotors->move(rightPower);

    pros::delay(10); // delay to save resources
  }

  // stop the drivetrain
  drivetrain.leftMotors->move(0);
  drivetrain.rightMotors->move(0);
  // set distTraveled to -1 to indicate that the function has finished
  distTravelled = -1;
  // give the mutex back
  mutex.give();
}
