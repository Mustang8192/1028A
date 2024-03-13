#include "main.h"
#include "1028A/api.h"

void initialize() { _1028A::utils::init(); }

void disabled() {
  _1028A::logger::info("Robot Disabled");
  if (climb == up) {
    _1028A::robot::climb_set1.set_value(0);
    _1028A::robot::climb_set2.set_value(1);
    climb = down;
  }
}

void competition_initialize() {}

void autonomous() { _1028A::comp::auton::auton(); }

void opcontrol() {
  _1028A::logger::info("Driver Control Enabled");
  _1028A::task::Async driveCTRL(_1028A::comp::driver::driveCTRL);
  _1028A::task::Async intakeCTRL(_1028A::comp::driver::intakeCTRL);
  _1028A::task::Async kickerCTRL(_1028A::comp::driver::kickerCTRL);
  _1028A::task::Async flapCTRL(_1028A::comp::driver::flapCTRL);
  _1028A::task::Async climbCTRL(_1028A::comp::driver::climbCTRL);
  //_1028A::task::Async loginputs(_1028A::comp::driver::logInputs);

  while (1) {
    pros::delay(200);
  }
}
