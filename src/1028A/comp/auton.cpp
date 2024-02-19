#include "1028A/comp/auton.h"
#include "1028A/legacy.h"
#include "1028A/robot.h"
#include "1028A/task.h"
#include "1028A/vars.h"
#include "pros/rtos.hpp"

void _1028A::comp::auton::auton() {
  // autonSelect = 2;
  if (autonSelect == 1) {
    robot::flapR.set_value(1);
    pros::delay(300);
    robot::flapR.set_value(0);
    robot::intake.move(-127);
    legacy::forward(750, 127, 1, 1200, 0, 0);
    pros::delay(200);
    legacy::turn(100, 127, 1, 1000, 0, 0);
    legacy::forward(200, 127, 1, 500, 0, 0);
    pros::delay(200);
    robot::intake.move(127);
    legacy::forward(-200, 127, 1, 500, 0, 0);
    legacy::turn(-40, 127, 1, 1000, 0, 0);
    robot::intake.move(-127);
    legacy::forward(200, 127, 1, 500, 0, 0);
    legacy::turn(100, 127, 1, 1000, 0, 0);
    legacy::forward(350, 127, 1, 500, 0, 0);
    legacy::turn(68, 127, 1, 500, 0, 0);
    robot::intake.move(127);
    legacy::forward(-200, 127, 1, 900, 0, 0);
    legacy::turn(-100, 127, 1, 1000, 0, 0);
    robot::intake.move(-127);
    legacy::forward(210, 127, 1, 900, 0, 0);
    pros::delay(200);
    legacy::forward(-350, 127, 1, 900, 0, 0);
    legacy::turn(100, 127, 1, 1000, 0, 0);
    robot::intake.move(127);
    pros::delay(200);
    robot::flapL.set_value(1);
    robot::flapR.set_value(1);
    legacy::forward(127, 700);
    legacy::forward(-300, 127, 1, 800, 0, 0);
    robot::flapR.set_value(0);
    robot::flapL.set_value(0);
    robot::intake.move(0);

  } else if (autonSelect == 2) {
    // 6 Ball
    // Grab Triball under pole
    robot::intake.move(-127);
    legacy::forward(60, 127, 3, 400, 0, 0);
    pros::delay(200);
    legacy::forward(-500, 127, 1, 1000, 0, 0);
    legacy::forward(40, 127, 3, 400, 0, 0);
    // Turn around to get Match Load
    legacy::turn(150, 127, 1, 1000, 0, 0);
    robot::intake.move(0);
    robot::flapL.set_value(1);
    legacy::forward(300, 127, 1, 500, 0, 0);
    robot::stick.set_value(1);
    legacy::forward(200, 127, 3, 600, 0, 0);
    legacy::turn(90, 127, 1, 700, 0, 0);
    legacy::forward(-200, 127, 3, 400, 0, 0);
    robot::stick.set_value(0);
    robot::intake.move(127);
    // Turn to score all 3 in goal
    legacy::turn(130, 127, 1, 600, 0, 0);
    robot::stick.set_value(0);
    robot::flapL.set_value(0);
    legacy::forward(127, 800);
    pros::delay(200);
    legacy::forward(-200, 127, 1, 300, 0, 0);
    legacy::turn(115, 127, 1, 300, 0, 0);
    legacy::forward(127, 800);
    legacy::forward(-250, 127, 1, 700, 0, 0);
    legacy::turn(17, 127, 1, 400, 0, 0);
    robot::intake.move(-127);
    legacy::forward(800, 127, 1, 1000, 0, 0);
    pros::delay(200);
    legacy::turn(130, 127, 1, 800, 0, 0);
    legacy::forward(290, 127, 1, 1000, 0, 0);
    robot::intake.move(127);
    pros::delay(200);
    legacy::forward(-100, 127, 1, 300, 0, 0);
    legacy::turn(54, 127, 1, 1000, 0, 0);
    robot::intake.move(-127);
    legacy::forward(310, 127, 1, 1000, 0, 0);
    legacy::turn(180, 127, 1, 800, 0, 0);
    robot::intake.move(127);
    robot::flapR.set_value(1);
    robot::flapL.set_value(1);
    Lwing = open;
    Rwing = open;
    legacy::forward(127, 1000);
  } else if (autonSelect == 3) {
    // Loading Rush
    robot::intake.move(-127);
    legacy::forward(730, 127, 1, 1200, 0, 0);
    legacy::forward(-750, 127, 1, 1000, 0, 0);
    pros::delay(1500);
    legacy::turn(80, 127, 1, 1000, 0, 0);
    robot::intake.move(127);
    legacy::forward(300, 127, 1, 1000, 0, 0);
  } else if (autonSelect == 4) {
    // Loading WP
    robot::stick.set_value(1);
    legacy::turn(-60, 127, 1, 1000, 0, 0);
    pros::delay(500);
    robot::stick.set_value(0);
    legacy::turn(-30, 127, 1, 1000, 0, 0);
    legacy::forward(580, 127, 1, 1400, 0, 0);

  } else if (autonSelect == 8) {
    // Skills
    legacy::forward(-127, 1000);
    legacy::forward(200, 127, 1, 500, 0, 0);
    legacy::ptturn(-90, 127, 1, 1000, 0, 0, true, false);
    legacy::ptturn(-112, 127, 1, 1000, 0, 0, false, true);
    robot::kicker.move(-127);
    legacy::forward(10, 2000);
    pros::delay(30000);
    robot::kicker.move(0);
    legacy::turn(10, 127, 1, 800, 0, 0);
    legacy::forward(340, 127, 1, 1000, -.1, 0.3);
    legacy::turn(65, 127, 0.5, 1000, 0, 0.1);
    legacy::forward(1280, 127, 1, 2000, -.2, 0.2);
    legacy::ptturn(125, 127, 1, 800, 0, 0, false, true);
    // robot::flapR.set_value(1);
    robot::intake.move(127);
    legacy::forward(127, 1000);
    legacy::forward(-100, 127, 1, 1000, 0, 0);
    legacy::forward(127, 1000);
    legacy::forward(-200, 127, 1, 1000, 0, 0);
    robot::flapR.set_value(0);
    robot::intake.move(0);
    legacy::turn(230, 127, 1, 1000, 0, 0);
    legacy::forward(750, 127, 1, 1000, 0, 0);
    legacy::turn(155, 127, 1, 1000, 0, 0);
    legacy::forward(300, 127, 1, 1000, 0, 0);
    legacy::turn(70, 127, 1, 1000, 0, 0);
    robot::flapL.set_value(1);
    robot::flapR.set_value(1);
    robot::intake.move(127);
    legacy::forward(127, 1500);
    legacy::forward(-200, 127, 1, 1000, 0, 0);
    legacy::forward(127, 1000);
    robot::flapL.set_value(0);
    robot::flapR.set_value(0);
    legacy::forward(-580, 127, 1, 1000, 0, 0);
    robot::intake.move(0);
    legacy::turn(155, 127, 1, 1000, 0, 0);
    legacy::forward(500, 127, 1, 1000, 0, 0);
    legacy::turn(20, 127, 1, 1000, 0, 0);
    robot::flapL.set_value(1);
    robot::flapR.set_value(1);
    robot::intake.move(127);
    legacy::forward(127, 1000);
    legacy::forward(-200, 127, 1, 1000, 0, 0);
    legacy::forward(127, 1000);
    robot::flapL.set_value(0);
    robot::flapR.set_value(0);
    legacy::forward(-300, 127, 1, 1000, 0, 0);
    legacy::turn(160, 127, 1, 1000, 0, 0);
    legacy::forward(400, 127, 1, 1000, 0, 0);
  }
}