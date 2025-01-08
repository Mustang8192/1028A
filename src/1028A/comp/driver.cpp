#include "1028A/comp/driver.h"
#include "1028A/misc/logger.h"
#include "1028A/misc/robot.h"
#include "1028A/misc/task.h"
#include "1028A/misc/vars.h"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include "1028A/misc/legacy.h"
#include <string>

void _1028A::comp::driver::driveCTRL(){

    _1028A::robot::leftfront.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    _1028A::robot::leftmid.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    _1028A::robot::leftback.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    _1028A::robot::rightfront.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    _1028A::robot::rightmid.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    _1028A::robot::rightback.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    _1028A::robot::intake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    _1028A::robot::LBL.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    _1028A::robot::LBR.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    while (1){
        int power = _1028A::robot::master.get_analog(ANALOG_LEFT_Y);
        int turn = _1028A::robot::master.get_analog(ANALOG_RIGHT_X);

        _1028A::robot::chassis.arcade(power, turn);
    
        pros::delay(10);
    }
}

void _1028A::comp::driver::intakeCTRL(){
    while (1){
        if (_1028A::robot::master.get_digital(DIGITAL_L1) && _1028A::robot::master.get_digital(DIGITAL_L2)){
            _1028A::robot::intake.move(127);
        }
        else if (_1028A::robot::master.get_digital(DIGITAL_L1)){
            _1028A::robot::intake.move(-127);
        }
        else if (_1028A::robot::master.get_digital(DIGITAL_L2)){
            _1028A::robot::intake.move(127);
        }
        else if (_1028A::robot::master.get_digital(DIGITAL_LEFT)){
            _1028A::robot::intake.move(40);
        }
        else if (_1028A::robot::master.get_digital(DIGITAL_RIGHT)){
            _1028A::robot::intake.move(-40);
        }
        else{
            _1028A::robot::intake.move(0);
        }
        pros::delay(10);
    }
}

void _1028A::comp::driver::mogoCTRL(){
    int status = 0;
    while (1){
        if (_1028A::robot::master.get_digital(DIGITAL_R1) && status == 0) {
            _1028A::robot::mogo.set_value(true);
            status = 1;
            pros::delay(300);
        }
        else if (_1028A::robot::master.get_digital(DIGITAL_R1) && status == 1){
            _1028A::robot::mogo.set_value(false);
            status = 0;
            pros::delay(300);
        }

        pros::delay(20);
    }
}

void _1028A::comp::driver::HSCTRL(){
   while(1){
      if (_1028A::robot::master.get_digital(DIGITAL_UP)){
        _1028A::robot::LB.move(127);
      }
      else if (_1028A::robot::master.get_digital(DIGITAL_DOWN)){
        _1028A::robot::LB.move(-127);
      }
      else if (_1028A::robot::master.get_digital(DIGITAL_LEFT)){
        _1028A::robot::LB.move(40);
      }
      else if (_1028A::robot::master.get_digital(DIGITAL_RIGHT)){
        _1028A::robot::LB.move(-40);
      }
      else{
        _1028A::robot::LB.move(0);
      }
    pros::delay(10);
   }
}


int requestedValue = 0;
void lbStatus(){
  int macroStat = 0;
  while (1){
    if(_1028A::robot::master.get_digital(DIGITAL_R1) && macroStat == 0){
      requestedValue = 10;
      macroStat = 1;
      pros::delay(300);
    }
    else if (_1028A::robot::master.get_digital(DIGITAL_R1) && macroStat == 1){
      requestedValue = 0;
      while (1){
        if (!_1028A::robot::master.get_digital(DIGITAL_R1)){
          macroStart = 0;
          break;
        }
        pros::delay(10);
      }
    }
    else{

    }
    
    pros::delay(20);
  }
} 

void _1028A::comp::driver::lbMacro(){
    float SensorCurrentValue;
  float error;
  float lastError = 0;
  bool works = true;
  float Kp = 1;
  float Ki = 0;
  float Kd = 0;
  double timeExit = 0;
  double startTime = pros::millis();
  pros::Task lbStatus(lbStatus);
  _1028A::robot::LBS.reset();

  while (1) {
    std::string message =
            std::to_string(_1028A::robot::LBS.get_position());
        std::cout << message << std::endl;

    // Reads the sensor value and scale
    SensorCurrentValue = _1028A::robot::LBS.get_position();
    // calculates error
    error = (requestedValue - SensorCurrentValue);
    // calculate drive PID
    float powerValue = _1028A::legacy::math(error, lastError, Kp, Ki, Kd, 127);

    if (fabs(powerValue) < 1) {
      _1028A::robot::LB.move(0);
      break;
    }

    // Move Motors with PID
    if(!works){
      int oldval = SensorCurrentValue;
      requestedValue = 0;
      _1028A::robot::LB.move(-127);
      while (1){
        if (_1028A::robot::LBSwitch.get_value() == 1){
          _1028A::robot::LB.move(0);
          requestedValue = oldval;
          break;
        }
        pros::delay(10);
      }
    }
    else{
    _1028A::robot::LB.move(powerValue);
    }

    lastError = error;
    pros::delay(5);
  }
}

void _1028A::comp::driver::macros() {
    while (1){
        if (1) {
        std::string message =
            "(" + std::to_string(robot::chassis.getPose().x) + ", " +
            std::to_string(robot::chassis.getPose().y) + ", " +
            std::to_string(robot::chassis.getPose().theta) + ")";
        std::cout << message << std::endl;
        }
    pros::delay(300);
  }
}