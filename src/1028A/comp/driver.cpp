#include "1028A/comp/driver.h"
#include "1028A/misc/robot.h"
#include "1028A/misc/task.h"

void _1028A::comp::driver::driverCTRL() {
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
      _1028A::robot::intakeMtrs.move(127);
    } else if (_1028A::robot::master.get_digital(DIGITAL_L2)) {
      _1028A::robot::intakeMtrs.move(-127);
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