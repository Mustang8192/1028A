#include "main.h"
#include "1028A/api.h"

void initialize() {
  _1028A::utils::init();
  _1028A::robot::chassis.calibrate();
}

void disabled() { _1028A::logger::info("Robot Disabled"); }

void competition_initialize() {}

void autonomous() { _1028A::comp::auton::auton(); }

void opcontrol() {
  _1028A::logger::info("Driver Control Enabled");
  _1028A::task::Async driveCTRL(_1028A::comp::driver::driveCTRL);
  _1028A::task::Async intakeCTRL(_1028A::comp::driver::intakeCTRL);
  _1028A::task::Async ptoCTRL(_1028A::comp::driver::ptoCTRL);
  while (1) {
    pros::delay(20);
  }
}
