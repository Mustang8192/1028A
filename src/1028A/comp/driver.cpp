#include "1028A/comp/driver.h"
#include "1028A/misc/logger.h"
#include "1028A/misc/robot.h"
#include "1028A/misc/task.h"
#include "1028A/misc/vars.h"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include <string>

void _1028A::comp::driver::driverCTRL() {
  autonSelect = 0;
  robot::optical.set_led_pwm(100);
  robot::leftfront.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  robot::leftmid.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  robot::leftback.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  robot::rightfront.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  robot::rightmid.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  robot::rightback.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  robot::intakeL.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  robot::intakeR.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  while (1) {

    int power = _1028A::robot::master.get_analog(ANALOG_LEFT_Y);
    int turn = _1028A::robot::master.get_analog(ANALOG_RIGHT_X);

    _1028A::robot::leftMtrs.move(power + turn);
    _1028A::robot::rightMtrs.move(power - turn);

    pros::delay(10);
  }
}

void _1028A::comp::driver::intakeCTRL() {
  while (1) {
    if (_1028A::robot::master.get_digital(DIGITAL_L1)) {
     _1028A::robot::intakeMtrs.move(-127);
     macroStart = 0;
    } else if (_1028A::robot::master.get_digital(DIGITAL_L2)) {
      _1028A::robot::intakeMtrs.move(127);
      macroStart = 0;
    } else if (_1028A::robot::master.get_digital(DIGITAL_LEFT)) {
      _1028A::robot::intakeMtrs.move(40);
      macroStart = 0;
    } else if (_1028A::robot::master.get_digital(DIGITAL_RIGHT)) {
      _1028A::robot::intakeMtrs.move(-40);
      macroStart = 0;
    } else if (_1028A::robot::master.get_digital(DIGITAL_R2)) {
      macroStart = 1;
      pros::delay(300);
    } 
    pros::delay(10);
  }
}

void _1028A::comp::driver::mogoCTRL() {
  int status = 0;
  while (1) {
    if (_1028A::robot::master.get_digital(DIGITAL_R1) && status == 0) {
      _1028A::robot::mogo.set_value(true);
      status = 1;
      pros::delay(300); 
    } else if (_1028A::robot::master.get_digital(DIGITAL_R1) && status == 1) {
      _1028A::robot::mogo.set_value(false);
      status = 0;
      pros::delay(300);
    }

    pros::delay(20);
  }
}

int Delay = 0;
int offset = 0;

void _1028A::comp::driver::HGCTRL(){
  while (1){
    if (macroStart == 1){
      _1028A::robot::intakeMtrs.move(127);
      while (1){
        if (robot::ringL.get()<20){
          _1028A::robot::intakeMtrs.move(60);
          while (1){
            if (robot::ring.get()<25){
              pros::delay(Delay + offset);
              _1028A::robot::intakeMtrs.move(0);
              pros::delay(300);
              _1028A::robot::intakeMtrs.move(-80);
              pros::delay(200);
              _1028A::robot::intakeMtrs.move(0);
              macroStart = 0;
            }
            else if (!macroStart){
              break;
            }
            pros::delay(10);
          }
        }
        else if (!macroStart){
              break;
            }
        pros::delay(10);
      }
    }
    else if (!macroStart && (!_1028A::robot::master.get_digital(DIGITAL_L1) && !_1028A::robot::master.get_digital(DIGITAL_L2) && !_1028A::robot::master.get_digital(DIGITAL_LEFT) && !_1028A::robot::master.get_digital(DIGITAL_RIGHT))) {
      _1028A::robot::intakeMtrs.move_velocity(0);
    }
    pros::delay(20);
  }
}

void _1028A::comp::driver::trimCTRL(){
  while (1){
    if (robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
      offset += 30;
      pros::delay(500);
    } else if (robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
      offset -= 30;
      pros::delay(500);
    }
    robot::master.print(1, 1, "trim: %i      ", offset);
    
    pros::delay(20);
  }
}

void _1028A::comp::driver::assistance() {
  while (1) {
    /*
    if (robot::conveyor.get_actual_velocity() == 0 &&
        robot::conveyor.is_stopped() &&
        (_1028A::robot::master.get_digital(DIGITAL_L2) or
         _1028A::robot::master.get_digital(DIGITAL_A))) {
      robot::master.rumble("-");
      robot::master.print(1, 1, "Lock up");
    } else {
      robot::master.clear_line(1);
    }
    */
    pros::delay(20);
  }
}

void _1028A::comp::driver::stickCTRL(){
  int toggle = 0;
  while (1){
    if (robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_A) && toggle == 0){
      toggle = 1;
      robot::stick.set_value(1);
      pros::delay(300);
    } else if (robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_A) && toggle == 1){
      toggle = 0;
      robot::stick.set_value(0);
      pros::delay(300);
    }
    pros::delay(20);
  }
}

void _1028A::comp::driver::macros() {
  while (1) {
    if (startLogging) {
      logger::info("Input Logging Started");
      pros::delay(3000);
      robot::master.rumble("..");
      int mogo;
      while (1) {
        if (!startLogging) {
          break;
        }
        int leftFrontpwr = robot::leftfront.get_target_velocity();
        int leftMidpwr = robot::leftmid.get_target_velocity();
        int leftBackpwr = robot::leftback.get_target_velocity();
        int rightFrontpwr = robot::rightfront.get_target_velocity();
        int rightMidpwr = robot::rightmid.get_target_velocity();
        int rightBackpwr = robot::rightback.get_target_velocity();
        int intakepwr = robot::intakeL.get_target_velocity();
        int conveyorpwr = robot::intakeR.get_target_velocity();

        if (robot::master.get_digital(DIGITAL_R1)) {
          int mogo = 1;
        } else {
          int mogo = 0;
        }

        std::string data[9] = {
            std::to_string(leftFrontpwr), std::to_string(leftMidpwr),
            std::to_string(leftBackpwr),  std::to_string(rightFrontpwr),
            std::to_string(rightMidpwr),  std::to_string(rightBackpwr),
            std::to_string(intakepwr),    std::to_string(conveyorpwr),
            std::to_string(mogo)};
        // convert array to string
        std::string str = "";
        for (int i = 0; i < 8; i++) {
          str += data[i] + ",";
        }
        str = "{" + str + "},";

        printf("%s", str.c_str());
        pros::delay(20);
      }

    } else {
    }

    if (startReadout) {
      std::string message =
          "(" + std::to_string(robot::chassis.getPose().x) + ", " +
          std::to_string(robot::chassis.getPose().y) + ", " +
          std::to_string(robot::chassis.getPose().theta) + ")";
      logger::info(message.c_str());
    }

    pros::delay(300);
  }
}