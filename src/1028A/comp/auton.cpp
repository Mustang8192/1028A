#include "1028A/comp/auton.h"
#include "1028A/legacy.h"
#include "1028A/robot.h"
#include "1028A/vars.h"
#include "main.h"
#include "pros/apix.h"
#include "pros/rtos.hpp"

void _1028A::comp::auton::auton() {
  if (autonSelect == 1) {
    // Snatch
    robot::flapR.set_value(1);
    robot::intake.move(-127);
    robot::intake.move(-127);
    robot::intake.move(-127);
    legacy::forward(800, 127, 4, 1000, -.2, .4);
    robot::flapR.set_value(0);
    pros::delay(500);
    legacy::turn(105, 127, 1, 700, 0, 0);
    pros::delay(100);
    legacy::forward(170, 127, 4, 500, -.2, .4);
    pros::delay(500);
    robot::intake.move(127);
    pros::delay(500);
    robot::intake.move(0);
    legacy::forward(-110, 127, 4, 1000, 0, 0);
    legacy::turn(-44, 127, 1, 1500, 0, 0);
    robot::intake.move(-127);
    legacy::forward(230, 127, 4, 1000, 0, 0);
    pros::delay(500);
    legacy::forward(-240, 127, 4, 1000, 0, 0);
    pros::delay(300);
    legacy::turn(110, 127, 1, 1000, 0, 0);
    legacy::forward(240, 127, 4, 1000, 0, 0);
    robot::intake.move(127);
    pros::delay(300);
    legacy::forward(-170, 127, 4, 1000, 0, 0);
    pros::delay(300);
    legacy::turn(-103, 127, 1, 1400, 0, 0);
    robot::intake.move(-127);
    pros::delay(100);
    legacy::forward(350, 127, 4, 1000, 0, 0);
    pros::delay(800);
    legacy::forward(-270, 127, 4, 1000, 0, 0);
    pros::delay(400);
    legacy::turn(97, 127, 1, 1000, 0, 0);
    robot::intake.move(127);
    pros::delay(300);
    robot::flapR.set_value(1);
    robot::flapL.set_value(1);
    legacy::forward(127, 800);
    legacy::forward(-250, 127, 1, 800, 0, 0);
  } else if (autonSelect == 4) {
    // Loading WP
    legacy::forward(660, 127, 4, 1000, 0, 0);
    pros::delay(300);
    legacy::turn(-90, 127, 1, 1000, 0, 0);
    robot::intake.move(127);
    pros::delay(300);
    legacy::forward(127, 600);
    pros::delay(300);
    legacy::forward(-400, 127, 1, 1000, 0, 0);
    robot::intake.move(0);
    pros::delay(300);
    legacy::turn(38, 127, 1, 1000, 0, 0);
    pros::delay(300);
    legacy::forward(-820, 127, 4, 1300, 0, 0);
    pros::delay(300);
    legacy::turn(134, 127, 1, 1000, 0, 0);
    robot::flapR.set_value(1);
    legacy::forward(200, 90, 1, 2000, 0, 0);
    pros::delay(200);
    legacy::turn(90, 127, 1, 1000, 0, 0);
    pros::delay(300);
    robot::flapR.set_value(1);
    legacy::forward(300, 127, 4, 2000, 0, 0);
    pros::delay(300);
    robot::flapR.set_value(0);
    legacy::turn(-90, 127, 1, 1000, 0, 0);
    pros::delay(300);
    legacy::forward(-230, 127, 4, 1000, 0, 0);
  } else if (autonSelect == 8) {
    // Skills
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
  }
}