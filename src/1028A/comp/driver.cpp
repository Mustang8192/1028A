#include "1028A/comp/driver.h"
#include "1028A/misc.h"
#include "1028A/robot.h"
#include "1028A/vars.h"
#include "pros/misc.h"
#include "pros/rtos.hpp"

void _1028A::comp::driver::driveCTRL() {
  while (1) {
    int power = _1028A::robot::master.get_analog(ANALOG_LEFT_Y);
    int turn = _1028A::robot::master.get_analog(ANALOG_RIGHT_X);

    _1028A::robot::leftMtrs.move(power + turn);
    _1028A::robot::rightMtrs.move(power - turn);

    pros::delay(5);
  }
}

void _1028A::comp::driver::flywheelCTRL() {
  while (1) {

    pros::delay(20);
  }
}

void _1028A::comp::driver::flapCTRL() {
  while (1) {
    int LSts = 0;
    int RSts = 0;
    int Bsts = 0;
    int actuations = 0;
    while (1) {
      if (robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
        if (RSts == 0) {
          robot::flapR.set_value(1);
          RSts = 1;
          actuations += 1;
          pros::delay(200);
        } else if (RSts == 1) {
          robot::flapR.set_value(0);
          RSts = 0;
          pros::delay(200);
        }
      }

      if (robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
        if (LSts == 0) {
          robot::flapL.set_value(1);
          LSts = 1;
          actuations += 1;
          pros::delay(200);
        } else if (LSts == 1) {
          robot::flapL.set_value(0);
          LSts = 0;
          pros::delay(200);
        }
      }

      if (robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
        if (LSts != RSts) {
          robot::flapL.set_value(0);
          robot::flapR.set_value(0);
          LSts = 0;
          RSts = 0;
          Bsts = 0;
          actuations += 1;
          pros::delay(200);
        } else if (LSts == 0 && RSts == 0) {
          robot::flapL.set_value(1);
          robot::flapR.set_value(1);
          LSts = 1;
          RSts = 1;
          Bsts = 1;
          actuations += 2;
          pros::delay(200);
        } else if (RSts == 1 && LSts == 1) {
          robot::flapL.set_value(0);
          robot::flapR.set_value(0);
          LSts = 0;
          RSts = 0;
          Bsts = 0;
          pros::delay(200);
        } else {
        }
      }

      robot::master.print(1, 1, "Act: %d", actuations);
      pros::delay(20);
    }
  }
}

void _1028A::comp::driver::intakeCTRL() {
  while (1) {
    if (_1028A::robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
      _1028A::robot::intake.move(127);

    } else if (_1028A::robot::master.get_digital(
                   pros::E_CONTROLLER_DIGITAL_L2)) {
      _1028A::robot::intake.move(-127);
    } else {
      _1028A::robot::intake.move(0);
    }
    pros::delay(10);
  }
}