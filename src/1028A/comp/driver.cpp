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
    if (PTO == ptoState::cata) {

      _1028A::robot::leftMtrs.move(power + turn);
      _1028A::robot::rightMtrs.move(power - turn);

      pros::delay(5);
    } else if (PTO == ptoState::drive) {
      _1028A::robot::leftMtrswPTO.move(power + turn);
      _1028A::robot::rightMtrswPTO.move(power - turn);

      pros::delay(5);
    }
  }
}

void _1028A::comp::driver::ptoCTRL() {
  while (1) {
    if (_1028A::robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
      _1028A::utils::ptoSwitch();
    }
    pros::delay(20);
  }
}

void _1028A::comp::driver::intakeCTRL() {
  while (1) {
    if (_1028A::robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      _1028A::robot::intake.move(127);
    } else if (_1028A::robot::master.get_digital(
                   pros::E_CONTROLLER_DIGITAL_R2)) {
      _1028A::robot::intake.move(-127);
    } else {
      _1028A::robot::intake.move(0);
    }
    pros::delay(10);
  }
}