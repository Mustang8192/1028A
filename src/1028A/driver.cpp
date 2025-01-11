#include "1028A/driver.h"
#include "1028A/legacy.h"
#include "1028A/logger.h"
#include "1028A/robot.h"
#include "pros/misc.h"
#include "pros/rtos.hpp"

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
void _1028A::driver::intakeCTRL() {
  while (1) {
    if (wallstake == 1) {
      _1028A::robot::intake.move(-127);
    } else if (_1028A::robot::master.get_digital(
                   pros::E_CONTROLLER_DIGITAL_L1)) {
      _1028A::robot::intake.move(-127);
    } else if (_1028A::robot::master.get_digital(
                   pros::E_CONTROLLER_DIGITAL_L2)) {
      _1028A::robot::intake.move(127);
    } else {
      _1028A::robot::intake.move(0);
    }

    pros::delay(15);
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


void armTask() {
  double rotationValue = 0;
  double armP = 0.0005;
  double armD = 0.565;
  double armError = 0;
  double armPrevError = 0;
  double threshold = 0.5;
  double armSpeed = 0;
  double rotationRaw = 0;

  _1028A::robot::LBS.set_reversed(4);

  while (true) {
    rotationRaw = -_1028A::robot::LBS.get_position();
    rotationValue = rotationRaw / 100;

    armError = armTarget - rotationValue;
    armSpeed = (armError * armP) + (armPrevError * armD);

    // lB.move(armSpeed);

    if (fabs(armError) <= threshold) {
      armSpeed = 0;
    }

    _1028A::robot::LB.move(armSpeed);

    armPrevError = armError;

    pros::delay(10);
  }
}

void _1028A::driver::odomRead (){
    while (1){
        if (0){
            std::string data = "(" + std::to_string(_1028A::robot::chassis.getPose().x) + ", " + std::to_string(_1028A::robot::chassis.getPose().y) + ", " + std::to_string(_1028A::robot::chassis.getPose().theta) + ")";
            _1028A::logger::info(data.c_str());
        }
        pros::delay(300);
    }
}
void _1028A::driver::lbmacro() {
  _1028A::robot::LB.move(-20);
  while (1) {
    if (_1028A::robot::LBSLimit.get_value()) {
      _1028A::robot::LBS.reset_position();
      pros::delay(100);
      _1028A::robot::LB.move(0);
      break;
    }
    pros::delay(20);
  }
  pros::Task arm(armTask);
  const int holdThreshold = 100;
    bool buttonPressed = false;
    bool longPressTriggered = false;
    int pressStartTime = 0;
    int loadPosition = 162;
  while (1) {
    if (robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            if (!buttonPressed) {
                buttonPressed = true;
                pressStartTime = pros::millis();
                longPressTriggered = false;
            }
            if (!longPressTriggered && pros::millis() - pressStartTime >= holdThreshold) {
                armTarget = 630;
                longPressTriggered = true;
            }
        } else {
            if (buttonPressed) {
                if (longPressTriggered) {
                    armTarget = 0;
                } else {
                    if (armTarget == loadPosition){
                        armTarget = 0;
                    }
                    else{
                        armTarget = loadPosition;
                    }
                }
                buttonPressed = false;
            }
        }
        pros::delay(20);
  }
}

