#include "main.h"
#include "1028A/api.h"

void initialize() {
  _1028A::utils::init();
  _1028A::robot::chassis.calibrate();
}

void disabled() { _1028A::robot::mogo.set_value(0); }

void competition_initialize() {}

ASSET(path1_txt);
void autonomous() { pros::delay(2000);
  

  
   }

void opcontrol() {
  _1028A::task::Async DriveCTRL(_1028A::comp::driver::driverCTRL);
  _1028A::task::Async IntakeCTRL(_1028A::comp::driver::intakeCTRL);
  _1028A::task::Async MogoCTRL(_1028A::comp::driver::mogoCTRL);
  _1028A::task::Async HGCTLR(_1028A::comp::driver::HGCTRL);
  _1028A::task::Async Assistance(_1028A::comp::driver::assistance);
  _1028A::task::Async Macros(_1028A::comp::driver::macros);
  while (true) {
    pros::delay(200);
  }
}
