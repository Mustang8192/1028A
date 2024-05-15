#include "main.h"
#include "1028A/api.h"

void initialize() {
  _1028A::utils::init();
  _1028A::robot::chassis.calibrate();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {

  while (true) {
    pros::delay(20);
  }
}
