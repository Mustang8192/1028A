#include "1028A/chassis/chassis.hpp"
#include <algorithm>
#include <math.h>

namespace _1028A {

float defaultDriveCurve(float input, float scale) {
  if (scale != 0) {
    return (powf(2.718, -(scale / 10)) + powf(2.718, (fabs(input) - 127) / 10) *
                                             (1 - powf(2.718, -(scale / 10)))) *
           input;
  }
  return input;
}

void Chassis::tank(int left, int right, float curveGain) {
  drivetrain.leftMotors->move(driveCurve(left, curveGain));
  drivetrain.rightMotors->move(driveCurve(right, curveGain));
}

void Chassis::arcade(int throttle, int turn, float curveGain) {
  int leftPower = driveCurve(throttle + turn, curveGain);
  int rightPower = driveCurve(throttle - turn, curveGain);
  drivetrain.leftMotors->move(leftPower);
  drivetrain.rightMotors->move(rightPower);
}

void Chassis::curvature(int throttle, int turn, float curveGain) {
  // If we're not moving forwards change to arcade drive
  if (throttle == 0) {
    arcade(throttle, turn, curveGain);
    return;
  }

  float leftPower = throttle + (std::abs(throttle) * turn) / 127.0;
  float rightPower = throttle - (std::abs(throttle) * turn) / 127.0;

  leftPower = driveCurve(leftPower, curveGain);
  rightPower = driveCurve(rightPower, curveGain);

  drivetrain.leftMotors->move(leftPower);
  drivetrain.rightMotors->move(rightPower);
}
} // namespace _1028A
