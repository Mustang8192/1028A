#include "1028A/comp/auton.h"
#include "1028A/legacy.h"
#include "1028A/robot.h"
#include "1028A/vars.h"
#include "main.h"

void _1028A::comp::auton::auton() {
  autonSelect = 1;
  if (autonSelect == 1) {
    /*
    legacy::forward(600, 127, 4, 3000, 0, 0.05);
    legacy::turn(84, 127, 1, 1000, 0, 0);
    robot::leftfront.move(127);
    robot::leftback.move(127);
    robot::leftmid.move(127);
    robot::rightfront.move(127);
    robot::rightback.move(127);
    robot::rightmid.move(127);
    pros::delay(400);
    robot::leftfront.brake();
    robot::leftback.brake();
    robot::leftmid.brake();
    robot::rightfront.brake();
    robot::rightback.brake();
    robot::rightmid.brake();
    legacy::forward(-220, 127, 4, 4000, 0, 0.05);
    legacy::turn(-101, 127, 1, 1000, 0, 0);
    pros::delay(150);
    robot::intake.move(-127);
    legacy::forward(375, 60, 4, 2000, 0, 0.05);
    pros::delay(400);
    robot::intake.move(-60);
    legacy::turn(-10, 127, 1, 1000, 0, 0);
    pros::delay(200);
    legacy::forward(175, 227, 4, 3000, 0, 0.05);
    legacy::turn(80, 127, 1, 2000, 0, 0);
    robot::flapL.set_value(1);
    robot::leftfront.move(127);
    robot::leftback.move(127);
    robot::leftmid.move(127);
    robot::rightfront.move(127);
    robot::rightback.move(127);
    robot::rightmid.move(127);
    pros::delay(800);
    robot::leftfront.brake();
    robot::leftback.brake();
    robot::leftmid.brake();
    robot::rightfront.brake();
    robot::rightback.brake();
    robot::rightmid.brake();
    pros::delay(200);
    robot::flapL.set_value(0);
    robot::intake.move(0);
    legacy::forward(-200, 127, 4, 1000, 0, 0);
    legacy::turn(-62, 127, 1, 800, 0, 0);
    pros::delay(500);
    robot::intake.move(-127);
    legacy::forward(370, 90, 4, 900, 0, 0);
    pros::delay(1000);
    robot::intake.move(-80);
    legacy::forward(-250, 127, 4, 1000, 0, 0);
    robot::leftfront.brake();
    robot::leftback.brake();
    robot::leftmid.brake();
    robot::rightfront.brake();
    robot::rightback.brake();
    robot::rightmid.brake();
    pros::delay(500);
    robot::flapL.set_value(1);
    legacy::turn(80, 127, 1, 1000, 0, 0);
    robot::leftfront.move(127);
    robot::leftback.move(127);
    robot::leftmid.move(127);
    robot::rightfront.move(127);
    robot::rightback.move(127);
    robot::rightmid.move(127);
    pros::delay(400);
    robot::leftfront.brake();
    robot::leftback.brake();
    robot::leftmid.brake();
    robot::rightfront.brake();
    robot::rightback.brake();
    robot::rightmid.brake();
    pros::delay(200);
    robot::flapL.set_value(0);
    legacy::forward(-400, 127, 4, 1500, 0, 0);
    */

  } else if (autonSelect == 2) {
    // Goal WP

  } else if (autonSelect == 8) {
    // Skills
  }
}