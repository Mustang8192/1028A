#include "1028A/misc/legacy.h"
#include "1028A/misc/logger.h"
#include "1028A/misc/robot.h"
#include "1028A/misc/task.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"

int _1028A::legacy::math(float Error, float lastError, float Kp, float Ki,
                         float Kd, double maxSpd) {
  float P;
  float D;
  float Drive;

  P = (Kp * Error);
  static float I = 0;
  I += Error * Ki;
  if (I > 1) {
    I = 1;
  }
  if (I < -1) {
    I = -1;
  }
  D = (Error - lastError) * Kd;
  Drive = P + I + D;

  if (Drive > maxSpd) {
    Drive = maxSpd;
  }
  if (Drive < -maxSpd) {
    Drive = -maxSpd;
  }
  return Drive;
}
bool _1028A::legacy::exit(float Error, float Threshold, float currTime,
                          float startTime, float timeExit, float powerValue,
                          float lastError) {
  if ((Error < Threshold and Error > -Threshold) && powerValue <= 5) {
    return true;
  } else if (currTime - startTime >= timeExit) {
    return true;
  } else {
    return false;
  }
}

void _1028A::legacy::turn(double RequestedValue, double spd, double thre,
                          double time, double kpOffset, double kdOffset) {
  float SensorCurrentValue;
  float error;
  float lastError = 0;

  float Kp = 1 + kpOffset;
  float Ki = 0;
  float Kd = 5 + kdOffset;
  double timeExit = 0;
  double startTime = pros::millis();
  _1028A::robot::leftfront.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  _1028A::robot::leftback.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  _1028A::robot::leftmid.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  _1028A::robot::rightfront.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  _1028A::robot::rightback.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  _1028A::robot::rightmid.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  robot::master.clear_line(1);
  while (1) {
    // Reads the sensor value and scale
    SensorCurrentValue = _1028A::robot::inertial.get_rotation();
    double currentTime = pros::millis();
    std::string print = "Turning: " + std::to_string(SensorCurrentValue);
    _1028A::logger::info(print.c_str());

    // calculates error
    error = -(RequestedValue - SensorCurrentValue);

    // calculate drive PID
    float powerValue = math(error, lastError, Kp, Ki, Kd, spd);
    if (exit(error, thre, currentTime, startTime, time, powerValue,
             lastError)) {
      _1028A::robot::leftfront.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      _1028A::robot::leftmid.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      _1028A::robot::leftback.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      _1028A::robot::rightfront.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      _1028A::robot::rightmid.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      _1028A::robot::rightback.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

      _1028A::robot::leftfront.brake();
      _1028A::robot::leftmid.brake();
      _1028A::robot::leftback.brake();
      _1028A::robot::rightfront.brake();
      _1028A::robot::rightmid.brake();
      _1028A::robot::rightback.brake();
      break;
    }
    // Move Motors with PID
    _1028A::robot::leftfront.move((0 * powerValue) + (-1 * powerValue));
    _1028A::robot::leftback.move((0 * powerValue) + (-1 * powerValue));
    _1028A::robot::leftmid.move((0 * powerValue) + (-1 * powerValue));

    _1028A::robot::rightfront.move((0 * powerValue) + (1 * powerValue));
    _1028A::robot::rightback.move((0 * powerValue) + (1 * powerValue));
    _1028A::robot::rightmid.move((0 * powerValue) + (1 * powerValue));

    lastError = error;
    pros::delay(5);
  }
  robot::master.print(1, 1, "done");
}

void _1028A::legacy::ptturn(double RequestedValue, double spd, double minspd,
                            double thre, double time, double kpOffset,
                            double kdOffset, bool leftLock, bool rightLock) {
  float SensorCurrentValue;
  float error;
  float lastError = 0;

  float Kp = 1 + kpOffset;
  float Ki = 0;
  float Kd = 5 + kdOffset;
  double timeExit = 0;
  double startTime = pros::millis();
  _1028A::robot::leftfront.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  _1028A::robot::leftback.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  _1028A::robot::leftmid.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  _1028A::robot::rightfront.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  _1028A::robot::rightback.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  _1028A::robot::rightmid.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  robot::master.clear_line(1);
  while (1) {
    // Reads the sensor value and scale
    SensorCurrentValue = _1028A::robot::inertial.get_rotation();
    double currentTime = pros::millis();
    std::string print = "Turning: " + std::to_string(SensorCurrentValue);
    _1028A::logger::info(print.c_str());

    // calculates error
    error = -(RequestedValue - SensorCurrentValue);

    // calculate drive PID
    float powerValue = math(error, lastError, Kp, Ki, Kd, spd);
    if (powerValue < minspd && powerValue > 0) {
      powerValue = minspd;
    }
    if (exit(error, thre, currentTime, startTime, time, powerValue,
             lastError)) {
      _1028A::robot::leftfront.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      _1028A::robot::leftmid.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      _1028A::robot::leftback.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      _1028A::robot::rightfront.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      _1028A::robot::rightmid.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      _1028A::robot::rightback.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

      _1028A::robot::leftfront.brake();
      _1028A::robot::leftmid.brake();
      _1028A::robot::leftback.brake();
      _1028A::robot::rightfront.brake();
      _1028A::robot::rightmid.brake();
      _1028A::robot::rightback.brake();
      break;
    }
    // Move Motors with PID
    if (!leftLock) {
      _1028A::robot::leftfront.move((0 * powerValue) + (-1 * powerValue));
      _1028A::robot::leftback.move((0 * powerValue) + (-1 * powerValue));
      _1028A::robot::leftmid.move((0 * powerValue) + (-1 * powerValue));
    }

    if (!rightLock) {
      _1028A::robot::rightfront.move((0 * powerValue) + (1 * powerValue));
      _1028A::robot::rightback.move((0 * powerValue) + (1 * powerValue));
      _1028A::robot::rightmid.move((0 * powerValue) + (1 * powerValue));
    }

    lastError = error;
    pros::delay(5);
  }
  robot::master.print(1, 1, "done");
}

void _1028A::legacy::forward(double RequestedValue, double spd, double thre,
                             double time, double kpOffset, double kdOffset) {
  float SensorCurrentValue;
  float error;
  float lastError = 0;

  float Kp = 0.6 + kpOffset;
  float Ki = 0;
  float Kd = 2.1 + kdOffset;
  double timeExit = 0;
  double startTime = pros::millis();
  _1028A::robot::leftmid.tare_position();
  while (1) {
    // Reads the sensor value and scale
    SensorCurrentValue = _1028A::robot::leftmid.get_position();
    double currentTime = pros::millis();
    std::string print = "Forward: " + std::to_string(SensorCurrentValue);
    _1028A::logger::info(print.c_str());

    // calculates error
    error = (RequestedValue - SensorCurrentValue);

    // calculate drive PID
    float powerValue = math(error, lastError, Kp, Ki, Kd, spd);

    if (exit(error, thre, currentTime, startTime, time, powerValue,
             lastError)) {
      _1028A::robot::leftfront.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      _1028A::robot::leftmid.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      _1028A::robot::leftback.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      _1028A::robot::rightfront.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      _1028A::robot::rightmid.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      _1028A::robot::rightback.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

      _1028A::robot::leftfront.brake();
      _1028A::robot::leftmid.brake();
      _1028A::robot::leftback.brake();
      _1028A::robot::rightfront.brake();
      _1028A::robot::rightmid.brake();
      _1028A::robot::rightback.brake();
      break;
    }

    // Move Motors with PID
    _1028A::robot::leftfront.move((0 * powerValue) + (1 * powerValue));
    _1028A::robot::leftback.move((0 * powerValue) + (1 * powerValue));
    _1028A::robot::leftmid.move((0 * powerValue) + (1 * powerValue));

    _1028A::robot::rightfront.move((0 * powerValue) + (1 * powerValue));
    _1028A::robot::rightback.move((0 * powerValue) + (1 * powerValue));
    _1028A::robot::rightmid.move((0 * powerValue) + (1 * powerValue));

    lastError = error;
    pros::delay(5);
  }
}

void _1028A::legacy::forward(double RequestedValue, double angle, double spd,
                             double thre, double time, double kpOffset,
                             double kdOffset) {
  float SensorCurrentValue;
  float angleSensor;
  float error;
  float angleerror;
  float lastError = 0;
  float lastangleerror = 0;

  float Kp = 0.6 + kpOffset;
  float Ki = 0;
  float Kd = 1.9 + kdOffset;

  float Ap = 1;
  float Ai = 0;
  float Ad = 5;

  double timeExit = 0;
  double startTime = pros::millis();
  _1028A::robot::leftmid.tare_position();
  while (1) {
    // Reads the sensor value and scale
    SensorCurrentValue = _1028A::robot::leftmid.get_position();
    angleSensor = _1028A::robot::inertial.get_rotation();
    double currentTime = pros::millis();
    std::string print = "Forward: " + std::to_string(SensorCurrentValue);
    _1028A::logger::info(print.c_str());

    // calculates error
    error = (RequestedValue - SensorCurrentValue);
    angleerror = -(angle - angleSensor);

    // calculate drive PID
    float powerValue = math(error, lastError, Kp, Ki, Kd, spd);
    float apowerValue = math(angleerror, lastangleerror, Ap, Ai, Ad, 50);

    if (exit(error, thre, currentTime, startTime, time, powerValue,
             lastError)) {
      _1028A::robot::leftfront.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      _1028A::robot::leftmid.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      _1028A::robot::leftback.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      _1028A::robot::rightfront.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      _1028A::robot::rightmid.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      _1028A::robot::rightback.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

      _1028A::robot::leftfront.brake();
      _1028A::robot::leftmid.brake();
      _1028A::robot::leftback.brake();
      _1028A::robot::rightfront.brake();
      _1028A::robot::rightmid.brake();
      _1028A::robot::rightback.brake();
      break;
    }

    // Move Motors with PID
    _1028A::robot::leftfront.move((-1 * apowerValue) + (1 * powerValue));
    _1028A::robot::leftback.move((-1 * apowerValue) + (1 * powerValue));
    _1028A::robot::leftmid.move((-1 * apowerValue) + (1 * powerValue));

    _1028A::robot::rightfront.move((1 * apowerValue) + (1 * powerValue));
    _1028A::robot::rightback.move((1 * apowerValue) + (1 * powerValue));
    _1028A::robot::rightmid.move((1 * apowerValue) + (1 * powerValue));

    lastError = error;
    pros::delay(5);
  }
}

void _1028A::legacy::forward(double RequestedValue, double spd, double minspd,
                             double thre, double time, double kpOffset,
                             double kdOffset, bool brake) {
  float SensorCurrentValue;
  float error;
  float lastError = 0;

  float Kp = 0.6 + kpOffset;
  float Ki = 0;
  float Kd = 1.9 + kdOffset;
  double timeExit = 0;
  double startTime = pros::millis();
  _1028A::robot::leftfront.tare_position();
  while (1) {
    // Reads the sensor value and scale
    SensorCurrentValue = _1028A::robot::leftfront.get_position();
    double currentTime = pros::millis();
    std::string print = "Forward: " + std::to_string(SensorCurrentValue);
    _1028A::logger::info(print.c_str());

    // calculates error
    error = (RequestedValue - SensorCurrentValue);

    // calculate drive PID
    float powerValue = math(error, lastError, Kp, Ki, Kd, spd);
    if (powerValue < minspd && powerValue > 0) {
      powerValue = minspd;
    }

    if (exit(error, thre, currentTime, startTime, time, powerValue,
             lastError)) {
      break;
    }

    // Move Motors with PID
    _1028A::robot::leftfront.move((0 * powerValue) + (1 * powerValue));
    _1028A::robot::leftback.move((0 * powerValue) + (1 * powerValue));
    _1028A::robot::leftmid.move((0 * powerValue) + (1 * powerValue));

    _1028A::robot::rightfront.move((0 * powerValue) + (1 * powerValue));
    _1028A::robot::rightback.move((0 * powerValue) + (1 * powerValue));
    _1028A::robot::rightmid.move((0 * powerValue) + (1 * powerValue));

    lastError = error;
    pros::delay(5);
  }
}

void _1028A::legacy::forward(double spd, double time) {
  robot::leftfront.move(spd);
  robot::leftback.move(spd);
  robot::leftmid.move(spd);
  robot::rightfront.move(spd);
  robot::rightback.move(spd);
  robot::rightmid.move(spd);
  pros::delay(time);
  robot::leftfront.brake();
  robot::leftback.brake();
  robot::leftmid.brake();
  robot::rightfront.brake();
  robot::rightback.brake();
  robot::rightmid.brake();
}

void _1028A::legacy::slantR(double spd, double time) {
  robot::leftfront.move(spd * 3);
  robot::leftback.move(spd * 3);
  robot::leftmid.move(spd * 3);
  robot::rightfront.move(spd);
  robot::rightback.move(spd);
  robot::rightmid.move(spd);
  pros::delay(time);
  robot::leftfront.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  robot::leftback.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  robot::leftmid.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  robot::rightfront.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  robot::rightmid.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  robot::rightback.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  robot::leftfront.brake();
  robot::leftback.brake();
  robot::leftmid.brake();
  robot::rightfront.brake();
  robot::rightmid.brake();
  robot::rightback.brake();
}

void _1028A::legacy::slantL(double spd, double time) {
  robot::leftfront.move(spd);
  robot::leftback.move(spd);
  robot::leftmid.move(spd);
  robot::rightfront.move(spd * 3);
  robot::rightback.move(spd * 3);
  robot::rightmid.move(spd * 3);
  pros::delay(time);
  robot::leftfront.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  robot::leftback.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  robot::leftmid.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  robot::rightfront.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  robot::rightmid.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  robot::rightback.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  robot::leftfront.brake();
  robot::leftback.brake();
  robot::leftmid.brake();
  robot::rightfront.brake();
  robot::rightmid.brake();
  robot::rightback.brake();
}
