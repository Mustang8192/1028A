#include "1028A/comp/auton.h"
#include "1028A/legacy.h"
#include "1028A/robot.h"
#include "1028A/vars.h"
#include "main.h"

void _1028A::comp::auton::auton() {
  if (autonSelect == 1) {
    // Goal Side
    _1028A::legacy::forward(600, 127, 4, 3000, 0, 0.05);
    _1028A::legacy::turn(90, 127, 1, 1000, 0, 0);
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
    _1028A::legacy::forward(-220, 127, 4, 4000, 0, 0.05);
    _1028A::legacy::turn(-101, 127, 1, 1000, 0, 0);
    pros::delay(150);
    robot::intake.move(-127);
    _1028A::legacy::forward(375, 60, 4, 2000, 0, 0.05);
    pros::delay(400);
    robot::intake.move(-60);
    _1028A::legacy::turn(-10, 127, 1, 1000, 0, 0);
    pros::delay(200);
    _1028A::legacy::forward(175, 227, 4, 3000, 0, 0.05);
    _1028A::legacy::turn(88, 127, 1, 2000, 0, 0);
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
    _1028A::legacy::forward(-200, 127, 4, 1000, 0, 0);
    _1028A::legacy::turn(-62, 127, 1, 800, 0, 0);
    pros::delay(500);
    robot::intake.move(-127);
    _1028A::legacy::forward(435, 90, 4, 900, 0, 0);
    pros::delay(1000);
    robot::intake.move(-80);
    _1028A::legacy::forward(-400, 127, 4, 1000, 0, 0);
    robot::leftfront.brake();
    robot::leftback.brake();
    robot::leftmid.brake();
    robot::rightfront.brake();
    robot::rightback.brake();
    robot::rightmid.brake();
    pros::delay(500);
    robot::flapL.set_value(1);
    _1028A::legacy::turn(80, 127, 1, 1000, 0, 0);
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
    _1028A::legacy::forward(-400, 127, 4, 1500, 0, 0);

  } else if (autonSelect == 2) {
    // Goal WP

  } else if (autonSelect == 8) {
    // Skills
  }
}