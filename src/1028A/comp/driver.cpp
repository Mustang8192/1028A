#include "1028A/comp/driver.h"
#include "1028A/legacy.h"
#include "1028A/logger.h"
#include "1028A/misc.h"
#include "1028A/robot.h"
#include "1028A/task.h"
#include "1028A/vars.h"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include <string>

void _1028A::comp::driver::driveCTRL() {
  // autonSelect = 12;
  if (autonSelect == 12) {
    legacy::slantR(-40, 800);
    legacy::forward(-127, 500);
    legacy::forward(140, 127, 1, 1000, 0, 0);
    legacy::ptturn(26, 127, 20, 1, 1500, 0, 0, false, true);
    robot::backL.set_value(1);
    robot::kicker.move(105);
    legacy::forward(-20, 1000);
    while (1) {
      if (_1028A::robot::master.get_analog(ANALOG_LEFT_Y) != 0 or
          _1028A::robot::master.get_analog(ANALOG_RIGHT_X) != 0) {
        robot::backL.set_value(0);
        robot::kicker.move(0);
        break;
      }

      if (_1028A::robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
        robot::leftfront.move(15);
        robot::leftmid.move(15);
        robot::leftback.move(15);
        robot::rightfront.move(-15);
        robot::rightmid.move(-15);
        robot::rightback.move(-15);
      } else if (_1028A::robot::master.get_digital(
                     pros::E_CONTROLLER_DIGITAL_LEFT)) {
        robot::leftfront.move(-15);
        robot::leftmid.move(-15);
        robot::leftback.move(-15);
        robot::rightfront.move(15);
        robot::rightmid.move(15);
        robot::rightback.move(15);
      } else {
        robot::leftfront.move(0);
        robot::leftmid.move(0);
        robot::leftback.move(0);
        robot::rightfront.move(0);
        robot::rightmid.move(0);
        robot::rightback.move(0);
      }

      pros::delay(20);
    }
  }
  autonSelect = NULL;
  robot::leftfront.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  robot::leftmid.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  robot::leftback.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  robot::rightfront.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  robot::rightmid.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  robot::rightback.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  _1028A::task::Async flapCTRL(_1028A::comp::driver::flapCTRL);
  while (1) {
    int power = _1028A::robot::master.get_analog(ANALOG_LEFT_Y);
    int turn = _1028A::robot::master.get_analog(ANALOG_RIGHT_X);

    if (driveSens == 0) {
      _1028A::robot::leftMtrs.move(power + turn);
      _1028A::robot::rightMtrs.move(power - turn);
    } else if (driveSens == 1) {
      _1028A::robot::leftMtrs.move(53);
      _1028A::robot::rightMtrs.move(53);
    }

    if (kickeron == 1) {
      robot::kicker.move(110);
    } else {
      robot::kicker.move(0);
    }

    // if (stickon == 1) {
    //   robot::stick.set_value(1);
    // } else {
    //  robot::stick.set_value(0);
    //}
    pros::delay(5);
  }
}

void _1028A::comp::driver::kickerCTRL() {
  while (1) {
    if (_1028A::robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) &&
        kickeron == 0) {
      kickeron = 1;
      pros::delay(400);
    } else if (_1028A::robot::master.get_digital(
                   pros::E_CONTROLLER_DIGITAL_R1) &&
               kickeron == 1) {
      kickeron = 0;
      pros::delay(400);
    }

    // if (_1028A::robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)
    // &&
    //     stickon == 0) {
    //   stickon = 1;
    //   pros::delay(300);
    // } else if (_1028A::robot::master.get_digital(
    //                pros::E_CONTROLLER_DIGITAL_RIGHT) &&
    //            stickon == 1) {
    //   stickon = 0;
    //   pros::delay(300);
    // }
    pros::delay(20);
  }
}

void _1028A::comp::driver::driveS() {
  while (1) {
    if (robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
      driveSens = 1;
    } else {
      driveSens = 0;
    }

    pros::delay(10);
  }
}

void _1028A::comp::driver::flapCTRL() {
  while (1) {
    if (robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
      if (Rwing == closed) {
        robot::flapR.set_value(1);
        Rwing = open;
        pros::delay(200);
      } else if (Rwing == open) {
        robot::flapR.set_value(0);
        Rwing = closed;
        pros::delay(200);
      }
    }

    if (robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
      if (Lwing == closed) {
        robot::flapL.set_value(1);
        Lwing = open;
        pros::delay(200);
      } else if (Lwing == open) {
        robot::flapL.set_value(0);
        Lwing = closed;
        pros::delay(200);
      }
    }

    if (robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
      if (Lwing != Rwing) {
        robot::flapL.set_value(1);
        robot::flapR.set_value(1);
        Lwing = open;
        Rwing = open;
        pros::delay(200);
      } else if (Lwing == Rwing && Lwing == closed) {
        robot::flapL.set_value(1);
        robot::flapR.set_value(1);
        Lwing = open;
        Rwing = open;
        pros::delay(200);
      } else if (Lwing == Rwing && Lwing == open) {
        robot::flapL.set_value(0);
        robot::flapR.set_value(0);
        Lwing = closed;
        Rwing = closed;
        pros::delay(200);
      } else {
      }
    }

    if (robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
      if (Rbwing == closed) {
        robot::backR.set_value(1);
        Rbwing = open;
        pros::delay(200);
      } else if (Rbwing == open) {
        robot::backR.set_value(0);
        Rbwing = closed;
        pros::delay(200);
      }
    }

    if (robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
      if (Lbwing == closed) {
        robot::backL.set_value(1);
        Lbwing = open;
        pros::delay(200);
      } else if (Lbwing == open) {
        robot::backL.set_value(0);
        Lbwing = closed;
        pros::delay(200);
      }
    }

    if (robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
      if (Lbwing != Rbwing) {
        robot::backL.set_value(1);
        robot::backR.set_value(1);
        Lbwing = open;
        Rbwing = open;
        pros::delay(200);
      } else if (Lbwing == Rbwing && Lbwing == closed) {
        robot::backL.set_value(1);
        robot::backR.set_value(1);
        Lbwing = open;
        Rbwing = open;
        pros::delay(200);
      } else if (Lbwing == Rbwing && Lbwing == open) {
        robot::backL.set_value(0);
        robot::backR.set_value(0);
        Lbwing = closed;
        Rbwing = closed;
        pros::delay(200);
      } else {
      }
    }
    pros::delay(20);
  }
}

void _1028A::comp::driver::intakeCTRL() {
  while (1) {
    if (_1028A::robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
      _1028A::robot::intake.move(-127);

    } else if (_1028A::robot::master.get_digital(
                   pros::E_CONTROLLER_DIGITAL_L2)) {
      _1028A::robot::intake.move(127);
    } else {
      _1028A::robot::intake.move(0);
    }
    pros::delay(10);
  }
}

void _1028A::comp::driver::climbCTRL() {
  while (1) {
    if (_1028A::robot::limitSwitch.get_value() == 1 && climb == up) {
      _1028A::robot::climb_set1.set_value(0);
      _1028A::robot::climb_set2.set_value(1);
      climb = down;
    } else if (_1028A::robot::master.get_digital(
                   pros::E_CONTROLLER_DIGITAL_R2) &&
               climb == neutral) {
      _1028A::robot::climb_set1.set_value(1);
      _1028A::robot::climb_set2.set_value(0);
      climb = up;
      pros::delay(300);
    } else if (_1028A::robot::master.get_digital(
                   pros::E_CONTROLLER_DIGITAL_R2) &&
               climb == up) {
      _1028A::robot::climb_set1.set_value(0);
      _1028A::robot::climb_set2.set_value(1);
      climb = down;
      pros::delay(300);
    } else if (_1028A::robot::master.get_digital(
                   pros::E_CONTROLLER_DIGITAL_R2) &&
               climb == down) {
      _1028A::robot::climb_set1.set_value(0);
      _1028A::robot::climb_set2.set_value(0);
      climb = neutral;
      pros::delay(300);
    }
    pros::delay(20);
  }
}

void _1028A::comp::driver::dataCTRL() {
  while (1) {
    if (overTemp == true) {
      _1028A::robot::master.print(0, 0, "Motors OverTemp");
    } else {
      if (Rwing == Lwing && Rwing == open) {
        _1028A::robot::master.print(0, 0, "Wing: Both");
      } else if (Lwing == open) {
        _1028A::robot::master.print(0, 0, "Wing: Left");
      } else if (Rwing == open) {
        _1028A::robot::master.print(0, 0, "Wing: Right");
      } else if (Lwing == closed && Rwing == closed) {
        _1028A::robot::master.print(0, 0, "Wing: None");
      }

      if (climb == up) {
        _1028A::robot::master.print(0, 10, "Climb: Up");
      } else if (climb == down) {
        _1028A::robot::master.print(0, 10, "Climb: Down");
      } else if (climb == neutral) {
        _1028A::robot::master.print(0, 10, "Climb: Neut");
      }
    }

    pros::delay(200);
  }
}

void _1028A::comp::driver::logInputs() {
  while (1) {
    if (startLogging) {
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
        int kickerpwr = robot::kicker.get_target_velocity();

        std::string data[8] = {
            std::to_string(leftFrontpwr), std::to_string(leftMidpwr),
            std::to_string(leftBackpwr),  std::to_string(rightFrontpwr),
            std::to_string(rightMidpwr),  std::to_string(rightBackpwr),
            std::to_string(intakepwr),    std::to_string(kickerpwr)};
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
      pros::delay(1000);
    }

    pros::delay(300);
  }
}