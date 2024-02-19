
#pragma once

#include "1028A/asset.hpp"
#include "1028A/chassis/trackingWheel.hpp"
#include "1028A/pose.hpp"
#include "pros/imu.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <functional>

namespace _1028A {
typedef struct {
  TrackingWheel *vertical1;
  TrackingWheel *vertical2;
  TrackingWheel *horizontal1;
  TrackingWheel *horizontal2;
  pros::Imu *imu;
} OdomSensors_t;

typedef struct {
  float kP;
  float kD;
  float smallError;
  float smallErrorTimeout;
  float largeError;
  float largeErrorTimeout;
  float slew;
} ChassisController_t;

typedef struct {
  pros::Motor_Group *leftMotors;
  pros::Motor_Group *rightMotors;
  float trackWidth;
  float wheelDiameter;
  float rpm;
  float chasePower;
} Drivetrain_t;

typedef std::function<float(float, float)> DriveCurveFunction_t;

float defaultDriveCurve(float input, float scale);

class Chassis {
public:
  Chassis(Drivetrain_t drivetrain, ChassisController_t lateralSettings,
          ChassisController_t angularSettings, OdomSensors_t sensors,
          DriveCurveFunction_t driveCurve = &defaultDriveCurve);
  void calibrate();
  void setPose(float x, float y, float theta, bool radians = false);
  void setPose(Pose pose, bool radians = false);
  Pose getPose(bool radians = false);
  Pose getSpeed(bool radians = false);
  Pose getLocalSpeed(bool radians = false);
  Pose estimatePose(float time, bool radians = false);
  void waitUntilDist(float dist);
  void turnTo(float x, float y, int timeout, bool async = false,
              bool reversed = false, float maxSpeed = 127, bool log = false);
  void moveTo(float x, float y, float theta, int timeout, bool async = false,
              bool forwards = true, float chasePower = 0, float lead = 0.6,
              float maxSpeed = 127, bool log = false);
  void follow(const asset &path, int timeout, float lookahead,
              bool async = false, bool forwards = true, float maxSpeed = 127,
              bool log = false);
  void tank(int left, int right, float curveGain = 0.0);
  void arcade(int throttle, int turn, float curveGain = 0.0);
  void curvature(int throttle, int turn, float cureGain = 0.0);

private:
  pros::Mutex mutex;
  float distTravelled = 0;

  ChassisController_t lateralSettings;
  ChassisController_t angularSettings;
  Drivetrain_t drivetrain;
  OdomSensors_t odomSensors;
  DriveCurveFunction_t driveCurve;
};
} // namespace _1028A
