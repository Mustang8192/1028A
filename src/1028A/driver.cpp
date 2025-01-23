#include "1028A/driver.h"
#include "1028A/legacy.h"
#include "1028A/logger.h"
#include "1028A/robot.h"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include "1028A/auton.h"

void _1028A::driver::driveCTRL() {
  _1028A::robot::leftMtrs.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  _1028A::robot::rightMtrs.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  _1028A::robot::LB.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

  while (1) {
    int power =
        _1028A::robot::master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int turn =
        _1028A::robot::master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    _1028A::robot::chassis.arcade(power, turn);

    pros::delay(10);
  }
}

int wallstake = 0;
int pauseControl = 0;
int doubleMacro = 0;
void _1028A::driver::intakeCTRL(){ 
  while (1) {
    if (pauseControl != 1){

      if (wallstake == 1) {
        _1028A::robot::intake.move(-127);
      }else if (_1028A::robot::master.get_digital(
                    pros::E_CONTROLLER_DIGITAL_L1)) {
        _1028A::robot::intake.move(-127);
      } else if (_1028A::robot::master.get_digital(
                    pros::E_CONTROLLER_DIGITAL_L2)) {
        _1028A::robot::intake.move(127);
      } else {
        _1028A::robot::intake.move(0);
      }
    }

    pros::delay(20);
  }
}

void _1028A::driver::mogoCTRL() {
  int status = 0;
  while (1) {
    if (_1028A::robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) &&
        status == 0) {
      _1028A::robot::mogo.set_value(1);
      status = 1;
      pros::delay(300);
    } else if (_1028A::robot::master.get_digital(
                   pros::E_CONTROLLER_DIGITAL_R1) &&
               status == 1) {
      _1028A::robot::mogo.set_value(0);
      status = 0;
      pros::delay(300);
    }
    pros::delay(15);
  }
}
double armTarget = 0;
int settled = 0;
int Reset = 0;
int hasReset = 0;
void armTask() {
  double rotationValue = 0;
  double armP = 0.0005;
  double armD = 0.565;
  double armError = 0;
  double armPrevError = 0;
  double threshold = 0.5;
  double armSpeed = 0;
  double rotationRaw = 0;
  _1028A::robot::LB.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
  _1028A::robot::LBS.set_reversed(1);
  _1028A::robot::LB.move(-40);
      while (1){
        if (_1028A::robot::LBSLimit.get_value()){
          _1028A::robot::LBS.set_position(0);
          armTarget = 0;
          pros::delay(200);
          _1028A::robot::LB.move(0);
          Reset = 0;
          break;
        }
        pros::delay(20);
      }


  while (true) {
    rotationRaw = -_1028A::robot::LBS.get_position();
    rotationValue = rotationRaw / 100;

    armError = armTarget - rotationValue;
    armSpeed = (armError * armP) + (armPrevError * armD);

    // lB.move(armSpeed);

    if (fabs(armError) <= threshold) {
      armSpeed = 0;
      settled = 1;
    }
    else {
      settled = 0;
    }

    if (_1028A::robot::LBSLimit.get_value() && armTarget == 0 && settled == 1 && hasReset == 0){
      _1028A::robot::LBS.reset_position();
      hasReset = 1;
    }
    else{
      hasReset = 0;
    }

    if (Reset){
      _1028A::robot::LB.move(127);
      pros::delay(200);
      _1028A::robot::LB.move(-40);
      while (1){
        if (_1028A::robot::LBSLimit.get_value()){
          _1028A::robot::LBS.set_position(0);
          armTarget = 0;
          pros::delay(200);
          _1028A::robot::LB.move(0);
          Reset = 0;
          break;
        }
        pros::delay(20);
      }
    }
    else{
      _1028A::robot::LB.move(armSpeed);
    }

    armPrevError = armError;

    pros::delay(10);
  }
}

void _1028A::driver::odomRead (){
    while (1){
        if (1){
            std::string data = "(" + std::to_string(_1028A::robot::chassis.getPose().x) + ", " + std::to_string(_1028A::robot::chassis.getPose().y) + ", " + std::to_string(_1028A::robot::chassis.getPose().theta) + ")";
            _1028A::logger::info(data.c_str());
        }
        pros::delay(300);
    }
}

int rapidLoad = 0;
void rapidScore(){
  while (1){
    if (rapidLoad == 1){
      armTarget = 630;
      pros::delay(800);
      armTarget = 162;


    }
  }
}
void _1028A::driver::lbmacro() {
  pros::Task arm(armTask);
  pros::Task rapidscore(rapidScore);
  const int holdThreshold = 150;
    bool buttonPressed = false;
    bool longPressTriggered = false;
    int pressStartTime = 0;
    int loadPosition = 95;
    int waitPosition = 200;
    int scorePosition = 580;
    int overScorePosition = 820;

    int state = 0;

    int startTime = pros::millis();

    while (1){
  if (robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
              if (!buttonPressed) {
                  buttonPressed = true;
                  pressStartTime = pros::millis();
                  longPressTriggered = false;
              }
              //if (_1028A::robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
              //    armTarget = 0;
              //}
              if (pros::millis() - pressStartTime >= holdThreshold) {
                  if (_1028A::robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
                    pauseControl =1;
                    robot::intake.move(0);
                      armTarget = overScorePosition;
                      
                  }
                  else{
                      wallstake = 1;
                      pros::delay(20);
                      wallstake = 0;
                      armTarget = scorePosition;
                      longPressTriggered = true;
                  }
              }
              else{
                pauseControl = 0;
              }
          } else {
            
              if (buttonPressed) {
                  if (longPressTriggered) {
                    pauseControl = 0;
                    armTarget = loadPosition;
                      
                  } else {
                    pauseControl = 0;
                      if ((armTarget == loadPosition) or armTarget == loadPosition - 45){
                          armTarget = 0;
                      }
                      else{
                          armTarget = loadPosition;
                          
                      }
                  }
                  buttonPressed = false;
              }
          }

        if (_1028A::robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
          rapidLoad = 1;
        }
        else{
          rapidLoad = 0;
        }  

        if (_1028A::robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
          _1028A::robot::master.rumble("-");
          _1028A::robot::optical.set_led_pwm(100);
          armTarget = loadPosition;
          pauseControl = 1;
          robot::intake.move(127);
          pros::delay(900);
          while (1){
            if (_1028A::robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
              if (armTarget > 600){
                    armTarget = 0;
                  }
              doubleMacro = 0;
              pauseControl = 0;
              break;
            }

            if (robot::intake.get_actual_velocity() < 2){
              pros::delay(400);
              robot::intake.move(-127);
              pros::delay(10);
              robot::intake.move(0);
              armTarget = waitPosition;
              pros::delay(200);
              robot::intake.move(127);
              break;
            }
            pros::delay(20);
          }
          while (1){
            if (_1028A::robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_L2) or _1028A::robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
              if (armTarget > 600){
                    armTarget = 0;
                  }
              doubleMacro = 0;
              pauseControl = 0;
              break;
            }

            if (robot::optical.get_proximity() > 253){
               robot::intake.move(0);
            }
            if (_1028A::robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
              armTarget = scorePosition;
              pros::delay(400);
              armTarget = loadPosition;
              pauseControl = 1;
              pros::delay(500);
              robot::intake.move(127);
              pros::delay(300);
              while (1){
                if (_1028A::robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_L2) or _1028A::robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
                  if (armTarget > 600){
                    armTarget = 0;
                  }
                  doubleMacro = 0;
                  break;
                }

                if (robot::intake.get_actual_velocity() < 2){
                  pros::delay(300);
                  robot::intake.move(-127);
                  pros::delay(10);
                  robot::intake.move(0);
                  pauseControl = 0;
                  armTarget = scorePosition;
                  pros::delay(400);
                  armTarget = 0;
                  break;
                }
                pros::delay(20);
              }
              break;
            }
            pros::delay(20);
          }
        }
        else{
          _1028A::robot::optical.set_led_pwm(0);
        }
        

        if (_1028A::robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)){
            Reset = 1;
            _1028A::auton::autoRan = 1;
            pros::delay(20);
        }

        if (_1028A::robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
            wallstake = 1;
            pros::delay(20);
            wallstake = 0;
            armTarget = waitPosition;
            pros::delay(200);
        }
        pros::delay(20);
  }
}

