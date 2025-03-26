#include "1028A/legacy.h"
#include "1028A/robot.h"

int _1028A::legacy::math(float Error, float lastError, float Kp, float Ki, float Kd, double maxSpd){
    float P;
    float D;
    float Drive;

    P=(Kp*Error);
    static float I = 0;
    I += (Ki*Error);
    if(I>1){
        I=1;
    }
    else if(I<-1){
        I=-1;
    }

    D=(Kd*(Error-lastError));
    Drive=P+I+D;
    if(Drive>maxSpd){
        Drive=maxSpd;
    }
    else if(Drive<-maxSpd){
        Drive=-maxSpd;
    }
    return Drive;
}

bool _1028A::legacy::exit(float Error, float Threshold, float currTime, float startTime, float timeExit, float powerValue, float lastError){
    if(Error<Threshold && Error>-Threshold){
        if(currTime-startTime>timeExit){
            return true;
        }
    }
    else{
        startTime=currTime;
        lastError=Error;
    }
    return false;
}

void _1028A::legacy::turn(double RequestedValue, double spd, double thre,
                          double time, double kpOffset, double kdOffset) {
  float SensorCurrentValue;
  float error;
  float lastError = 0;

  float Kp = 1.2 + kpOffset;
  float Ki = 0;
  float Kd = 1.3 + kdOffset;
  double timeExit = 0;
  double startTime = pros::millis();
  _1028A::robot::leftMtrs.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
  robot::master.clear_line(1);
  while (1) {
    // Reads the sensor value and scale
    SensorCurrentValue = _1028A::robot::inertial.get_rotation();
    double currentTime = pros::millis();

    // calculates error
    error = -(RequestedValue - SensorCurrentValue);

    // calculate drive PID
    float powerValue = math(error, lastError, Kp, Ki, Kd, spd);
    if (exit(error, thre, currentTime, startTime, time, powerValue,
             lastError)) {
      robot::leftMtrs.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
      robot::rightMtrs.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);

        robot::leftMtrs.move(0);
        robot::rightMtrs.move(0);
      break;
    }
    // Move Motors with PID
    _1028A::robot::leftMtrs.move((0 * powerValue) + (-1 * powerValue));
    _1028A::robot::rightMtrs.move((0 * powerValue) + (1 * powerValue));

    lastError = error;
    pros::delay(5);
  }
}


void _1028A::legacy::forward(double RequestedValue, double spd, double thre,
                             double time, double kpOffset, double kdOffset) {
  float SensorCurrentValue;
  float error;
  float lastError = 0;

  float Kp = 1 + kpOffset;
  float Ki = 0;
  float Kd = 0 + kdOffset;
  double timeExit = 0;
  double startTime = pros::millis();
  _1028A::robot::leftFront.tare_position();
  while (1) {
    // Reads the sensor value and scale
    SensorCurrentValue = _1028A::robot::leftFront.get_position() * 100;
    double currentTime = pros::millis();

    // calculates error
    error = (RequestedValue - SensorCurrentValue);

    // calculate drive PID
    float powerValue = math(error, lastError, Kp, Ki, Kd, spd);

    if (exit(error, thre, currentTime, startTime, time, powerValue,
             lastError)) {
      _1028A::robot::leftMtrs.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
      _1028A::robot::rightMtrs.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);

      _1028A::robot::leftMtrs.brake();
      _1028A::robot::rightMtrs.brake();
      break;
    }

    // Move Motors with PID
    _1028A::robot::leftMtrs.move((0 * powerValue) + (1 * powerValue));
    _1028A::robot::rightMtrs.move((0 * powerValue) + (1 * powerValue));
    
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

  float Kp = 1 + kpOffset;
  float Ki = 0;
  float Kd = 6 + kdOffset;

  float Ap = .3;
  float Ai = 0;
  float Ad = 5;

  double timeExit = 0;
  double startTime = pros::millis();
  _1028A::robot::leftMid.tare_position();
  while (1) {
    // Reads the sensor value and scale
    SensorCurrentValue = _1028A::robot::leftMid.get_position() * 100;
    angleSensor = _1028A::robot::inertial.get_rotation();
    double currentTime = pros::millis();

    // calculates error
    error = (RequestedValue - SensorCurrentValue);
    angleerror = -(angle - angleSensor);

    // calculate drive PID
    float powerValue = math(error, lastError, Kp, Ki, Kd, spd);
    float apowerValue = math(angleerror, lastangleerror, Ap, Ai, Ad, 50);

    if (exit(error, thre, currentTime, startTime, time, powerValue,
             lastError)) {
      _1028A::robot::leftMtrs.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
      _1028A::robot::rightMtrs.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);

      _1028A::robot::leftMtrs.brake();
      _1028A::robot::rightMtrs.brake();
      break;
    }

    // Move Motors with PID
    _1028A::robot::leftMtrs.move((-1 * apowerValue) + (1 * powerValue));
    _1028A::robot::rightMtrs.move((1 * apowerValue) + (1 * powerValue));

    lastError = error;
    pros::delay(5);
  }
}
