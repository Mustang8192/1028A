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

void closerush() {
  while (1) {
    if (_1028A::robot::leftmid.get_position() >= 600) {
      _1028A::robot::intake.move(-127);
      pros::delay(200);
      break;
    }
    pros::delay(5);
  }
}

void closerushwingret() {
  while (1) {
    if (_1028A::robot::leftmid.get_position() >= 200) {
      _1028A::robot::flapL.set_value(0);
      pros::delay(200);
      break;
    }
  }
}

void skillswng() {
  while (1) {
    if (_1028A::robot::inertial.get_rotation() >= 270) {
      _1028A::robot::flapL.set_value(1);
      pros::delay(200);
      break;
    }

    pros::delay(5);
  }
}
void _1028A::comp::auton::auton() {
  // autonSelect = 12;
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
    task::Async ballDrop(closerush);
    legacy::forward(750, 0, 127, 1, 1000, 0, 0);
    pros::delay(200);
    legacy::forward(-710, 127, 1, 1000, 0, 0);
    legacy::ptturn(130, 127, 0, 1, 1000, 0, 0, true, false);
    legacy::forward(-150, 127, 1, 1000, 0, 0);
    robot::stick.set_value(1);
    pros::delay(500);
    legacy::turn(0, 127, 1, 1000, 0, 0);
    robot::stick.set_value(0);
    legacy::turn(85, 127, 1, 1000, 0, 0);
    robot::flapR.set_value(1);
    robot::intake.move(127);
    legacy::forward(680, 85, 127, 1, 1000, 0, 0);
    robot::flapR.set_value(0);
    robot::intake.move(0);
    legacy::forward(-700, 80, 127, 1, 1000, 0, 0);
    legacy::turn(110, 127, 1, 1000, 0, 0);
    // legacy::forward(-120, 127, 1, 1000, 0, 0);
    robot::flapR.set_value(1);
  } else if (autonSelect == 4) {
    // Loading WP
    robot::flapL.set_value(1);
    pros::delay(200);
    robot::flapL.set_value(0);
    legacy::turn(65, 127, 1, 1000, 0, 0);
    robot::stick.set_value(1);
    legacy::turn(-30, 127, 1, 1000, 0, 0);
    robot::stick.set_value(0);
    // legacy::turn(55, 127, 1, 1000, 0, 0);
    // legacy::forward(180, 127, 1, 1000, 0, 0);
    legacy::turn(30, 127, 1, 1000, 0, 0);
    legacy::forward(150, 127, 1, 1000, 0, 0);
    legacy::turn(-11, 127, 1, 1000, 0, 0);
    legacy::forward(530, 127, 1, 1000, -.3, 0.4);
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

  } else if (autonSelect == 6) {
    // Bryce
    robot::intake.move(-127);
    robot::flapL.set_value(1);
    task::Async wingRetact(closerushwingret);
    legacy::forward(750, 127, 1, 1000, 0, 0);
    pros::delay(200);
    legacy::forward(-40, 127, 1, 300, 0, 0);
    legacy::turn(80, 127, 1.5, 1000, 0, 0);
    robot::flapL.set_value(1);
    legacy::forward(485, 127, 1, 500, 0, 0);
    robot::intake.move(127);
    robot::flapL.set_value(0);
    legacy::forward(-485, 127, 1, 500, 0, 0);
    legacy::turn(2, 127, 1, 1000, 0, 0);
    legacy::forward(-790, 2, 127, 1, 1200, 0, 0);
    legacy::turn(105, 127, 1, 1000, 0, 0);
    legacy::forward(-230, 105, 127, 1, 1000, 0, 0);
    robot::stick.set_value(1);
    legacy::turn(-10, 127, 1, 1000, 0, 0);
    legacy::turn(140, 127, 1, 1000, 0, 0);
    robot::stick.set_value(0);
    legacy::forward(-600, 127, 1, 1500, -.5, 0.4);
    legacy::forward(400, 127, 1, 1000, 0, 0);
    robot::stick.set_value(1);
    legacy::turn(-10, 127, 1, 1000, 0, 0);
    legacy::turn(80, 127, 1, 1000, 0, 0);

  } else if (autonSelect == 12) {
    // Skills
    int startTime = pros::millis();
    legacy::slantR(-40, 800);
    legacy::forward(-127, 500);
    legacy::forward(200, 127, 1, 600, 0, 0);
    legacy::turn(-171, 127, 1, 800, 0, 0);
    robot::flapR.set_value(1);
    robot::kicker.move(-100);
    pros::delay(21500);
    robot::kicker.move(0);
    robot::flapR.set_value(0);
    legacy::turn(10, 127, 1, 1000, 0, 0);
    robot::flapL.set_value(1);
    legacy::forward(780, 127, 1, 1200, 0, 0);
    legacy::turn(90.5, 127, 1.5, 800, 0, 0);
    robot::flapL.set_value(1);
    robot::intake.move(127);
    legacy::forward(1120, 90.5, 127, 1, 1400, 0, -.4);
    robot::flapL.set_value(0);
    pros::delay(100);
    // legacy::turn(0, 127, 1, 1400, 0.8, 0);
    // legacy::forward(300, 127, 1, 1000, 0, 0);
    legacy::turn(0, 127, 1, 1400, 0.8, 0);
    legacy::forward(-640, 127, 1, 1000, 0, 0);
    legacy::turn(58, 127, 1, 700, 0, 0);
    legacy::forward(500, 127, 1, 600, 0, 0);
    legacy::turn(20, 127, 1, 800, 0, 0);
    legacy::forward(200, 127, 7, 400, 0, 0);
    robot::flapL.set_value(1);
    legacy::forward(800, 0, 127, 5, 800, 0, 0);
    // legacy::forward(200, 80, 5, 1000, 0, 0);
    robot::intake.move(127);
    legacy::turn(-13, 127, 1, 400, 0, 0);
    legacy::forward(530, 127, 1, 1000, 0, 0);
    legacy::turn(-46, 127, 1, 700, 0, 0);
    legacy::forward(127, 900);
    legacy::forward(-150, 127, 1, 600, 0, 0);
    robot::flapL.set_value(0);
    legacy::forward(127, 900);
    legacy::forward(-100, 127, 1, 600, 0, 0);
    legacy::turn(-90, 127, 2, 400, 0, 0);
    legacy::ptturn(-180, 127, 0, 1, 1000, 0, 0, false, true);

    legacy::forward(530, 127, 1, 800, 0, 0);
    legacy::ptturn(-10, 127, 0, 1, 800, 0, 0, false, true);
    robot::flapR.set_value(1);
    robot::intake.move(127);
    legacy::forward(80, 1100);
    robot::flapR.set_value(0);

    legacy::forward(-150, 127, 1, 600, 0, 0);
    robot::intake.move(0);
    // task::Async wingOpen(skillswng);
    legacy::turn(130, 127, 2, 800, 0, 0);
    legacy::forward(220, 127, 1, 1000, 0, 0);
    legacy::turn(230, 127, 1, 800, 0, 0);
    legacy::forward(180, 127, 1, 1000, 0.3, 0);
    legacy::ptturn(360, 127, 20, 1, 1000, 0.8, 0, false, true);
    // robot::flapL.set_value(0);
    robot::flapR.set_value(1);
    // robot::flapL.set_value(1);
    robot::intake.move(127);
    legacy::forward(70, 1000);
    // robot::flapL.set_value(0);
    robot::flapR.set_value(0);
    robot::flapL.set_value(0);
    pros::delay(700);
    legacy::forward(-450, 127, 1, 600, 0, 0);
    legacy::turn(270, 127, 1, 1000, 0, 0);
    legacy::forward(460, 127, 1, 1500, 0, 0);
    legacy::ptturn(350, 127, 20, 1, 1500, 0.9, 0, false, true);
    legacy::turn(423, 127, 1, 1500, 0, 0);
    robot::flapL.set_value(1);
    robot::flapR.set_value(1);
    legacy::forward(800, 370, 127, 1, 1500, 0, 0);
    legacy::forward(-200, 127, 1, 1000, 0, 0);
    legacy::turn(360, 127, 1, 1000, 0, 0);
    legacy::forward(127, 800);
    robot::flapL.set_value(0);
    robot::flapR.set_value(0);
    legacy::forward(-100, 127, 1, 1000, 0, 0);
    legacy::turn(270, 127, 1, 1000, 0, 0);
    legacy::forward(820, 127, 1, 1000, 0, 0);
    legacy::turn(370, 127, 1, 1000, 0, 0);
    robot::flapR.set_value(1);
    legacy::forward(600, 127, 5, 800, 0, 0);
    // legacy::turn(480, 127, 8, 400, 0, 0);
    legacy::forward(127, 1000);
    legacy::forward(-150, 127, 1, 1000, 0, 0);
    // legacy::turn(460, 127, 8, 1000, 0, 0);

    legacy::forward(127, 800);
    legacy::forward(-150, 127, 1, 1000, 0, 0);
    // legacy::turn(460, 127, 8, 1000, 0, 0);
    legacy::forward(127, 800);
    legacy::forward(-150, 127, 1, 1000, 0, 0);
    // legacy::turn(460, 127, 8, 1000, 0, 0);
    legacy::forward(127, 800);
    legacy::forward(-500, 127, 1, 1000, 0, 0);

    /*
  if (pros::millis() - startTime >= 50000) {
    legacy::forward(127, 1000);
    legacy::forward(-150, 127, 1, 1000, 0, 0);
    // legacy::turn(460, 127, 8, 1000, 0, 0);

    legacy::forward(127, 800);
    legacy::forward(-150, 127, 1, 1000, 0, 0);
    // legacy::turn(460, 127, 8, 1000, 0, 0);
    legacy::forward(127, 800);
    legacy::forward(-150, 127, 1, 1000, 0, 0);
    // legacy::turn(460, 127, 8, 1000, 0, 0);
    legacy::forward(127, 800);
    legacy::forward(-500, 127, 1, 1000, 0, 0);
  } else if (pros::millis() - startTime >= 55000) {
    legacy::forward(127, 1000);
    legacy::forward(-150, 127, 1, 1000, 0, 0);
    // legacy::turn(460, 127, 8, 1000, 0, 0);

    legacy::forward(127, 800);
    legacy::forward(-500, 127, 1, 1000, 0, 0);
  } else if (pros::millis() - startTime >= 58000) {
    legacy::forward(127, 1000);
    legacy::forward(-500, 127, 1, 1000, 0, 0);
  }
  */
    // legacy::turn(460, 127, 8, 1000, 0, 0)
  }
}