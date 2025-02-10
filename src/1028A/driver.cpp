#include "1028A/driver.h"
#include "1028A/legacy.h"
#include "1028A/logger.h"
#include "1028A/robot.h"
#include "1028A/ui.h"
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
    //_1028A::robot::leftMtrs.move(power + turn);
    //_1028A::robot::rightMtrs.move(power - turn);

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

void _1028A::driver::stickCTRL(){
  int status = 0;
  while (1) {
    if (_1028A::robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) &&
        status == 0) {
      _1028A::robot::stick.set_value(1);
      status = 1;
      pros::delay(300);
    } else if (_1028A::robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) &&
               status == 1) {
      _1028A::robot::stick.set_value(0);
      status = 0;
      pros::delay(300);
    }
    pros::delay(20);
  }
}
double armTarget = 0;
int settled = 0;
int Reset = 0;
int hasReset = 0;
int _1028A::driver::skills = 0;
void armTask() {
  double rotationValue = 0;
  double armP = 0.0005;
  double armD = 0.43;
  double armError = 0;
  double armPrevError = 0;
  double threshold = 0.5;
  double armSpeed = 0;
  double rotationRaw = 0;
  _1028A::robot::LB.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
  _1028A::robot::LBS.set_reversed(1);
  if (_1028A::driver::skills != 1){
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

    if (_1028A::robot::LBSLimit.get_value() && hasReset == 0){
      _1028A::robot::LBS.reset_position();
      hasReset = 1;
    }
    else{
      hasReset = 0;
    }
    
    if (Reset && _1028A::driver::skills == 1){
      _1028A::robot::LB.move(127);
      pros::delay(100);
      _1028A::robot::LB.move(-90);
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
    else if (Reset && _1028A::driver::skills == 0){
      _1028A::robot::LB.move(127);
      pros::delay(100);
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

int odomOverride = 0;
void _1028A::driver::odomRead (){
    while (1){
        if (_1028A::ui::callbacks::macros::recordPos or odomOverride or _1028A::robot::CaliSwitch.get_value()){
            std::string data = "(" + std::to_string(_1028A::robot::chassis.getPose().x) + ", " + std::to_string(_1028A::robot::chassis.getPose().y) + ", " + std::to_string(_1028A::robot::chassis.getPose().theta) + ")";
            _1028A::logger::info(data.c_str());
            _1028A::ui::callbacks::macros::recordPos = 0;
        }
        pros::delay(300);
    }
}
void _1028A::driver::lbmacro() {
  pros::Task arm(armTask);
  const int holdThreshold = 150;
    bool buttonPressed = false;
    bool longPressTriggered = false;
    int pressStartTime = 0;
    int offset = 0;
    int loadPosition = 110;
    int waitPosition = 200;
    int alliscorePosition = 500;
    int goalscorePosition = 620;
    int wallScorePosition = 310;
    int overScorePosition = 820;
    int wallOverScorePosition = 475;

    int state = 0;
    int scoring =0;
    int startTime = pros::millis();

    if (skills == 1){
    armTarget = 500;
    pros::delay(700);
    Reset = 1;
    skills = 0;
  }

    while (1){
      
          if (robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            _1028A::robot::intake.move(0);
            pauseControl =1;
              if (!buttonPressed) {
                pauseControl =1;
                  buttonPressed = true;
                  pressStartTime = pros::millis();
                  longPressTriggered = false;
              }
              if (pros::millis() - pressStartTime >= holdThreshold) {
                  if (_1028A::robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
                    pauseControl =1;
                    robot::intake.move(0);
                      armTarget = wallOverScorePosition + offset;
                      
                  }
                  else if (_1028A::robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
                    pauseControl =1;
                    robot::intake.move(0);
                      armTarget = overScorePosition;
                      
                  }

                  else{
                    if (scoring == 0){
                      wallstake = 1;
                      pros::delay(20);
                      wallstake = 0;
                      armTarget = wallScorePosition;
                      longPressTriggered = true;
                      scoring = 1;
                    }
                    else {
                      wallstake = 0;
                      armTarget = wallScorePosition;
                      longPressTriggered = true;
                    }
                  }
              }
              else{
                pauseControl = 0;
              }
          }
          else if (robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
              if (pros::millis() - pressStartTime >= holdThreshold) {
                 buttonPressed = 1;
                  if (_1028A::robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_L2) or _1028A::robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
                    pauseControl =1;
                    robot::intake.move(0);
                      armTarget = overScorePosition;
                      
                  }
                  else if (_1028A::robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
                    pauseControl =1;
                    robot::intake.move(0);
                      armTarget = goalscorePosition;
                      
                  }
                  else{
                    if (scoring == 0){
                      wallstake = 1;
                      pros::delay(20);
                      wallstake = 0;
                      armTarget = alliscorePosition;
                      //longPressTriggered = true;
                      scoring = 1;
                    }
                    else {
                      wallstake = 0;
                      armTarget = alliscorePosition;
                      //longPressTriggered = true;
                    }
                  }
              }
              else{
                pauseControl = 0;
              }
          } else {
            scoring = 0;
              if (buttonPressed) {
                  if (longPressTriggered) {
                    pauseControl = 0;
                    armTarget = loadPosition + offset;
                      
                  } else {
                    pauseControl = 0;
                      if ((armTarget == (loadPosition + offset)) or armTarget == loadPosition - 45){
                          armTarget = 0;
                      }
                      else{
                          armTarget = loadPosition + offset;
                          
                      }
                  }
                  buttonPressed = false;
              }
          }

        if (_1028A::robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
          _1028A::robot::master.rumble("-");
          _1028A::robot::optical.set_led_pwm(100);
          armTarget = loadPosition + offset;
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
              armTarget = (wallScorePosition + 180);
              pros::delay(900);
              armTarget = loadPosition;
              pauseControl = 1;
              pros::delay(300);
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
                  armTarget = (wallScorePosition + 170);
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
            pros::delay(20);
        }

        if (_1028A::robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
            wallstake = 1;
            pros::delay(20);
            wallstake = 0;
            armTarget = waitPosition;
            pros::delay(200);
        }

        if (_1028A::robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
            offset+=10;
            robot::master.rumble("-");
            pros::delay(200);
        }
        else if (_1028A::robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)){
            offset-=10;
            robot::master.rumble("-");
            pros::delay(200);
        }
    }
        pros::delay(20);
  }

