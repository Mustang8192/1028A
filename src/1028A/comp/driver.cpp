#include "1028A/comp/driver.h"
#include "1028A/misc/logger.h"
#include "1028A/misc/robot.h"
#include "1028A/misc/task.h"
#include "1028A/misc/vars.h"

void _1028A::comp::driver::driverCTRL() {
  autonSelect = 0;
  robot::leftfront.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  robot::leftmid.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  robot::leftback.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  robot::rightfront.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  robot::rightmid.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  robot::rightback.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
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
    } else if (_1028A::robot::master.get_digital(DIGITAL_L2)) {
      _1028A::robot::intakeMtrs.move(127);
    } else {
      _1028A::robot::intakeMtrs.move_velocity(0);
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
      pros::delay(200);
    } else if (_1028A::robot::master.get_digital(DIGITAL_R1) && status == 1) {
      _1028A::robot::mogo.set_value(false);
      status = 0;
      pros::delay(200);
    }

    pros::delay(20);
  }
}

void _1028A::comp::driver::assistance() {
  while (1) {
    if (robot::conveyor.get_actual_velocity() == 0 &&
        _1028A::robot::master.get_digital(DIGITAL_L2)) {
      robot::master.rumble(".");
      robot::master.print(1, 1, "Lock up");
    } else {
      robot::master.clear_line(1);
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
        int intakepwr = robot::intake.get_target_velocity();
        int conveyorpwr = robot::conveyor.get_target_velocity();

        std::string data[8] = {
            std::to_string(leftFrontpwr), std::to_string(leftMidpwr),
            std::to_string(leftBackpwr),  std::to_string(rightFrontpwr),
            std::to_string(rightMidpwr),  std::to_string(rightBackpwr),
            std::to_string(intakepwr),    std::to_string(conveyorpwr)};
        // convert array to string
        std::string str = "";
        for (int i = 0; i < 8; i++) {
          str += data[i] + ",";
        }
        str = "{" + str + "},";

        printf(str.c_str());
        pros::delay(20);
      }

    } else {
    }

    if (startReadout) {
      lemlib::Pose pose = robot::chassis.getPose();
      std::string message = "(" + std::to_string(pose.x) + ", " +
                            std::to_string(pose.y) + ", " +
                            std::to_string(pose.theta) + ")";
      logger::info(message.c_str());
    }

    pros::delay(300);
  }
}