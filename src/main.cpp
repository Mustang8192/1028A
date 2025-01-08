#include "main.h"
#include "1028A/api.h"

void initialize() {
  _1028A::utils::init();
  _1028A::robot::chassis.calibrate();
}

void disabled() { }

void competition_initialize() {}

void autonomous() { _1028A::comp::auton(); }

void opcontrol() {
  _1028A::task::Async DriveCTRL(_1028A::comp::driver::driveCTRL);
  _1028A::task::Async IntakeCTRL(_1028A::comp::driver::intakeCTRL);
  _1028A::task::Async MogoCTRL(_1028A::comp::driver::mogoCTRL);
  _1028A::task::Async Stick (_1028A::comp::driver::stickCTRL);
  _1028A::task::Async lbmacro(_1028A::comp::driver::lbMacro);
  _1028A::task::Async Macros(_1028A::comp::driver::macros);
  while (true) {
    pros::delay(200);
  }
}
