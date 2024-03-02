#include "1028A/comp/auton.h"
#include "1028A/legacy.h"
#include "1028A/robot.h"
#include "1028A/task.h"
#include "1028A/vars.h"
#include "pros/rtos.hpp"

void rush5balldrop() {
  while (1) {
    if (_1028A::robot::leftmid.get_position() <= -80) {
      _1028A::robot::intake.move(127);
      pros::delay(200);
      break;
    }
    pros::delay(5);
  }
}

void _1028A::comp::auton::auton() {
  autonSelect = 8;
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

  } else if (autonSelect == 5) {
    // Rush_6
    robot::intake.move(-127);
    legacy::forward(750, 127, 1, 1000, 0, 0);
    pros::delay(200);
    task::Async ballDrop(rush5balldrop);
    legacy::forward(-600, 100, 1, 800, 0, 0);
    // legacy::ptturn(-105, 127, 0, 1, 1000, 0, 0, false, true);
    legacy::ptturn(-90, 127, 0, 1, 500, 0, 0, false, true);
    // legacy::ptturn(-65, 127, 0, 1, 600, 0, 0, false, true);
    robot::intake.move(-127);
    legacy::forward(470, 100, 1, 800, 0, 0);
    pros::delay(200);
    legacy::turn(-58, 127, 1, 800, 0.4, 0);
    legacy::forward(-490, 127, 1, 1000, 0, 0);
    robot::intake.move(0);
    // Turn around to get Match Load
    legacy::turn(64, 127, 1, 1000, 0, 0);
    robot::flapL.set_value(1);
    legacy::forward(250, 127, 3, 600, 0, 0);
    robot::stick.set_value(1);
    legacy::ptturn(5, 127, 0, 1, 1000, 0.4, 0, true, false);
    robot::stick.set_value(0);
    legacy::ptturn(70, 127, 0, 1, 1000, 0.4, 0, true, false);
    legacy::forward(-200, 127, 3, 1000, 0, 0);
    robot::intake.move(127);
    legacy::forward(250, 127, 3, 1000, 0, 0);
    legacy::ptturn(10, 127, 0, 1, 1000, 0.4, 0, true, false);
    legacy::forward(450, 127, 3, 500, 0, 0);
    legacy::forward(-200, 127, 3, 1000, 0, 0);
    legacy::forward(450, 127, 3, 1000, 0, 0);
    robot::flapL.set_value(1);
    legacy::ptturn(60, 127, 0, 1, 1000, 0.4, 0, false, true);
    robot::intake.move(-127);
    legacy::forward(600, 127, 3, 1000, 0, 0);
    legacy::turn(88, 127, 1, 800, 0, 0);
    robot::flapR.set_value(1);
    robot::intake.move(127);
    legacy::forward(300, 127, 3, 1000, 0, 0);
    robot::flapR.set_value(0);
    legacy::slantR(-40, 800);
    robot::intake.move(-127);
    legacy::forward(300, 127, 3, 1000, 0, 0);
    legacy::turn(110, 127, 1, 800, 0, 0);
    robot::flapR.set_value(1);
    robot::flapL.set_value(1);
    legacy::forward(1000, 127, 3, 1000, 0, 0);

    /*legacy::forward(-200, 127, 3, 400, 0, 0);
    robot::stick.set_value(0);
    robot::intake.move(127);

    // Turn to score all 3 in goal
    legacy::turn(-50, 127, 1, 600, 0, 0);
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
    legacy::forward(127, 1000);*/

  } else if (autonSelect == 8) {
    // Skills
    legacy::slantR(-40, 800);
    legacy::forward(-127, 500);
    legacy::forward(200, 127, 1, 600, 0, 0);
    legacy::turn(-163, 127, 1, 800, 0, 0);
    robot::flapR.set_value(1);
    robot::kicker.move(-95);
    pros::delay(26000);
    robot::kicker.move(0);
    robot::flapR.set_value(0);
    legacy::turn(10, 127, 1, 1000, 0, 0);
    legacy::forward(740, 127, 1, 1200, 0, 0);
    legacy::turn(90, 127, 1.5, 800, 0, 0);
    robot::flapL.set_value(1);
    robot::intake.move(127);
    legacy::forward(1200, 90, 127, 1, 1400, 0, -.4);
    robot::flapL.set_value(0);
    pros::delay(300);
    legacy::turn(-3, 127, 1, 1000, 0.6, 0);
    legacy::forward(-700, 127, 1, 1000, 0, 0);
    legacy::turn(70, 127, 1, 700, 0, 0);
    legacy::forward(500, 127, 1, 600, 0, 0);
    legacy::turn(8, 127, 1, 1000, 0, 0);
    legacy::forward(200, 127, 3, 400, 0, 0);
    robot::flapL.set_value(1);
    legacy::forward(800, -4, 127, 5, 1000, 0, 0);
    // legacy::forward(200, 80, 5, 1000, 0, 0);
    robot::intake.move(127);
    legacy::turn(-8, 127, 1, 800, 0, 0);
    legacy::forward(450, 127, 1, 600, 0, 0);
    legacy::turn(-65, 127, 1, 700, 0, 0);
    legacy::forward(127, 900);
    legacy::forward(-150, 127, 1, 600, 0, 0);
    robot::flapL.set_value(0);
    legacy::forward(127, 900);
    legacy::ptturn(-180, 127, 0, 1, 1000, 0, 0, false, true);
    legacy::forward(500, 127, 1, 1000, 0, 0);
    legacy::ptturn(0, 127, 0, 1, 1500, 0, 0, false, true);
    robot::flapL.set_value(1);
    robot::intake.move(127);
    legacy::forward(127, 1000);
    robot::flapL.set_value(0);
    legacy::forward(-560, 127, 1, 1000, 0, 0);
    pros::delay(300);
    legacy::turn(-90, 127, 1, 1000, 0, 0);
    legacy::forward(400, 127, 1, 1000, 0, 0);
    legacy::turn(16.5, 127, 1, 1000, 0, 0);
    robot::flapL.set_value(1);
    robot::flapR.set_value(1);
    legacy::forward(127, 1000);
    robot::flapL.set_value(0);
    robot::flapR.set_value(0);
    legacy::forward(-100, 127, 1, 1000, 0, 0);
    legacy::turn(-90, 127, 1, 1000, 0, 0);
    legacy::forward(600, 127, 1, 1000, 0, 0);
    robot::flapL.set_value(1);
    legacy::ptturn(70, 127, 0, 1, 1000, 0, 0, false, true);
    robot::flapL.set_value(0);
    legacy::forward(127, 1000);
    legacy::forward(-350, 127, 1, 800, 0, 0);
    legacy::turn(215, 127, 1.5, 800, 0, 0);
    legacy::forward(400, 127, 1, 600, 0, 0);
    legacy::turn(175, 127, 1, 800, 0, 0);
    _1028A::robot::climb_set1.set_value(1);
    _1028A::robot::climb_set2.set_value(0);
    climb = up;
    legacy::forward(630, 127, 1, 1000, 0, 0);
    /*
    legacy::turn(360, 127, 1, 1000, 0, 0);
    legacy::forward(-550, 127, 1, 1000, 0, 0);
    legacy::turn(295, 127, 1, 1000, 0, 0);
    legacy::forward(127, 1000);
    */
  }
}