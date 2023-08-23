#include "1028A/comp/driver.h"
#include "1028A/robot.h"

void _1028A::comp::driver::driveCTRL() {
  while (1) {
    int power = _1028A::robot::master.get_analog(ANALOG_LEFT_Y);
    int turn = _1028A::robot::master.get_analog(ANALOG_RIGHT_X);

    _1028A::robot::leftfront.move(power + turn);
    _1028A::robot::leftmid.move(power + turn);
    _1028A::robot::leftback.move(power + turn);
    _1028A::robot::rightfront.move(power - turn);
    _1028A::robot::rightmid.move(power - turn);
    _1028A::robot::rightback.move(power - turn);

    pros::delay(5);
  }
}

void _1028A::comp::driver::ptoCTRL() {}

void _1028A::comp::driver::intakeCTRL() {}