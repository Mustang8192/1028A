#include "1028A/comp/auton.h"
#include "1028A/legacy.h"
#include "1028A/robot.h"
#include "1028A/vars.h"
#include "main.h"
#include "pros/rtos.hpp"

void _1028A::comp::auton::auton() {
  autonSelect = 1;
  if (autonSelect == 1) {
    robot::flapR.set_value(1);
    robot::intake.move(-127);
    legacy::forward(850, 127, 4, 1000, -.2, .4);
    robot::flapR.set_value(0);
    pros::delay(500);
    legacy::turn(92, 127, 1, 1000, 0, 0);
    pros::delay(300);
    legacy::forward(200, 127, 4, 1000, -.2, .4);
    pros::delay(500);
    robot::intake.move(127);
    pros::delay(500);
    robot::intake.move(0);
    legacy::forward(-100, 127, 4, 1000, 0, 0);
    legacy::turn(-47, 127, 1, 1000, 0, 0);
    robot::intake.move(-127);
    legacy::forward(237, 127, 4, 1000, 0, 0);
    pros::delay(500);
    legacy::forward(-210, 127, 4, 1000, 0, 0);
    pros::delay(300);
    legacy::turn(98, 127, 1, 1000, 0, 0);
    legacy::forward(270, 127, 4, 1000, 0, 0);
    robot::intake.move(127);
    pros::delay(300);
    legacy::forward(-100, 127, 4, 1000, 0, 0);
    pros::delay(300);
    legacy::turn(-105, 127, 1, 1400, 0, 0);
    robot::intake.move(-127);
    pros::delay(300);
    legacy::forward(440, 127, 4, 1000, 0, 0);
    pros::delay(500);
    legacy::forward(-310, 127, 4, 1000, 0, 0);
    pros::delay(300);
    legacy::turn(98, 127, 1, 1000, 0, 0);
    pros::delay(300);
    robot::intake.move(127);
    pros::delay(300);
    robot::flapR.set_value(1);
    robot::flapL.set_value(1);
    legacy::forward(127, 800);
    pros::delay(400);
    legacy::forward(-100, 127, 1, 800, 0, 0);
  } else if (autonSelect == 2) {
    // Goal WP

  } else if (autonSelect == 8) {
    robot::flywheel.move(127);
    pros::delay(40000);
    robot::flywheel.move(0);
    legacy::turn(-60, 127, 1, 1000, 0, 0);
    pros::delay(400);
    legacy::forward(270, 127, 4, 1000, 0, 0);
    pros::delay(400);
    legacy::turn(-16, 127, 1, 1000, 0, 0);
    legacy::forward(1150, 127, 4, 5000, 0, 0);
    pros::delay(500);
    legacy::turn(73, 127, 1, 1000, 0, 0);
    robot::flapR.set_value(1);
    legacy::forward(920, 127, 4, 5000, 0, 0);
    pros::delay(500);
    legacy::turn(-20, 127, 1, 1000, 0, 0);
    robot::flapL.set_value(1);
    robot::flapR.set_value(1);
    robot::intake.move(127);
    legacy::forward(127, 1000);
    legacy::forward(-400, 127, 4, 1000, 0, 0);
    legacy::forward(127, 1000);
    legacy::forward(-400, 127, 4, 1000, 0, 0);
    legacy::forward(127, 1000);
    legacy::forward(-400, 127, 4, 1000, 0, 0);
    legacy::forward(127, 1000);
    legacy::forward(-400, 127, 4, 1000, 0, 0);
    // Skills
  }
}