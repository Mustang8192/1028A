#include "main.h"
#include "1028A/api.h"

void initialize() {
  _1028A::utils::init();
  _1028A::robot::chassis.calibrate();
}

void disabled() { _1028A::robot::mogo.set_value(0); }

void competition_initialize() {}

void autonomous() { _1028A::comp::auton(); }

void opcontrol() {
  _1028A::task::Async DriveCTRL(_1028A::comp::driver::driverCTRL);
  _1028A::task::Async IntakeCTRL(_1028A::comp::driver::intakeCTRL);
  _1028A::task::Async MogoCTRL(_1028A::comp::driver::mogoCTRL);
  _1028A::task::Async IntakeLiftCTRL(_1028A::comp::driver::intakeLiftCTRL);
  _1028A::task::Async HgLiftCTRL(_1028A::comp::driver::hgLiftCTRL);
  _1028A::task::Async Assistance(_1028A::comp::driver::assistance);
  _1028A::task::Async Macros(_1028A::comp::driver::macros);
  while (true) {
    pros::delay(200);
  }
}
