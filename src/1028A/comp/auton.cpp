#include "1028A/comp/auton.h"
#include "1028A/legacy.h"
#include "1028A/robot.h"
#include "1028A/task.h"
#include "1028A/vars.h"
#include "main.h"
#include "pros/apix.h"
#include "pros/rtos.hpp"

void async() {
  if (autonSelect == 4) {
    while (1) {
      if (_1028A::robot::inertial.get_rotation() <= -150) {
        _1028A::robot::flapL.set_value(0);
      }

      pros::delay(20);
    }
  }
}

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
    legacy::turn(90, 127, 1, 1000, 0, 0);
    robot::intake.move(127);
    pros::delay(300);
    robot::flapR.set_value(1);
    robot::flapL.set_value(1);
    legacy::forward(127, 800);
    legacy::forward(-250, 127, 1, 800, 0, 0);
  } else if (autonSelect == 4) {
    // Loading WP
    robot::flapL.set_value(1);
    pros::delay(400);
    task::Async Async(async);
    legacy::turn(-480, 50, 1, 6000, 0, 0);
    Async.forceStop();
    pros::delay(500);
    legacy::forward(-230, 127, 1, 1000, 0, 0);
    pros::delay(500);
    legacy::turn(-525, 127, 1, 1000, 0, 0);
    pros::delay(500);
    legacy::forward(-438, 127, 4, 1000, 0, 0.4);
    /*
    robot::intake.move(127);
    robot::flapL.set_value(1);
    legacy::forward(127, 600);
    pros::delay(400);
    robot::flapL.set_value(0);
    legacy::turn(-430, 127, 1, 2000, 0, 0);
    legacy::forward(-300, 127, 4, 1000, 0, 0, false);
    robot::intake.move(0);
    legacy::forward(-70, 1200);
    legacy::turn(-500, 50, 1, 6000, 0, 0);
    legacy::forward(-450, 127, 4, 1000, 0, 0.4);
    */
  } else if (autonSelect == 8) {
    // Skills
    // robot::flywheel.move(127);
    // pros::delay(40000);
    /*
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
    */
    robot::flywheel.move(127);
    pros::delay(40000);
    robot::flywheel.move(0);
    legacy::turn(-59, 127, 1, 1000, 0, 0);
    legacy::forward(270, 127, 4, 1000, 0, 0);
    pros::delay(200);
    legacy::turn(-19, 127, 1, 1000, 0, 0);
    legacy::forward(1000, 127, 4, 5000, 0, 0);
    robot::flapL.set_value(1);
    legacy::ptturn(110, 127, 1, 1500, 0.2, 0.3, false, true);
    legacy::forward(640, 127, 1, 1000, 0, 0);
    legacy::ptturn(-34, 127, 1, 1500, 0, 0, true, false);
    robot::flapR.set_value(1);
    robot::intake.move(127);
    legacy::forward(127, 1000);
    legacy::forward(-400, 127, 4, 1000, 0, 0);
    legacy::forward(127, 1000);
    legacy::forward(-480, 127, 4, 1000, 0, 0);
    /*
    robot::flapL.set_value(0);
    robot::flapR.set_value(0);
    legacy::turn(-120, 127, 1, 1000, 0, 0);
    legacy::forward(-600, 127, 1, 1000, 0, 0);
    legacy::turn(-84, 127, 1, 1000, 0, 0);
    robot::flapL.set_value(1);
    robot::flapR.set_value(1);
    pros::delay(500);
    legacy::forward(300, 127, 1, 1000, 0, 0);
    pros::delay(400);
    legacy::ptturn(-34, 127, 1, 1000, 0, 0, false, true);
    legacy::forward(127, 1000);
    legacy::forward(-400, 127, 4, 1000, 0, 0);
    legacy::forward(127, 1000);
    legacy::forward(-480, 127, 4, 1000, 0, 0);
    /*

    legacy::forward(-127, 1000);
    legacy::forward(-80, 1000);
    pros::delay(500);
    legacy::forward(210, 127, 4, 800, 0, 0);
    pros::delay(200);
    legacy::turn(90, 127, 1, 1000, 0, 0);
    legacy::forward(-45, 900);
    robot::flywheel.move(127);
    pros::delay(35000);
    robot::flywheel.move(0);
    legacy::turn(20, 127, 1, 1000, 0, 0);
    legacy::forward(370, 127, 4, 1000, 0, 0, false);
    legacy::turn(76, 127, 1, 1000, 0, 0);
    legacy::forward(1200, 127, 4, 2300, 0, 0);
    pros::delay(500);
    robot::flapL.set_value(1);
    legacy::ptturn(205, 127, 1, 1000, 0.2, 0.3, false, true);
    robot::flapR.set_value(1);
    legacy::forward(430, 127, 4, 1000, 0, 0);
    robot::flapL.set_value(0);
    robot::intake.move(127);
    legacy::ptturn(71, 127, 1, 1000, 0.2, 0.3, true, false);
    robot::flapL.set_value(1);
    legacy::forward(127, 1000);
    pros::delay(500);
    legacy::forward(-400, 127, 4, 1000, 0, 0, false);
    legacy::forward(127, 1000);
    robot::intake.move(0);
    pros::delay(500);
    robot::flapL.set_value(0);
    robot::flapR.set_value(0);
    legacy::forward(-500, 127, 4, 1000, 0, 0);
    legacy::turn(-20, 127, 1, 1000, 0.2, 0.3);
    legacy::forward(-570, 127, 4, 1000, 0, 0);
    robot::flapL.set_value(1);
    robot::flapR.set_value(1);
    legacy::turn(10, 127, 1, 1300, 0, 0);
    legacy::forward(310, 127, 4, 1000, 0, 0, false);
    legacy::ptturn(70, 127, 1, 1000, 0.2, 0.3, false, true);
    robot::intake.move(127);
    legacy::forward(127, 1200);
    legacy::forward(-400, 127, 4, 1000, 0, 0);
    legacy::forward(127, 1200);
    legacy::forward(-400, 127, 4, 1000, 0, 0);
    /*
    legacy::turn(105, 127, 1, 1500, 0, 0);
    robot::flapR.set_value(1);
    legacy::forward(200, 127, 4, 1000, 0, 0, false);
    legacy::turn(155, 127, 1, 1500, 0, 0);
    legacy::forward(127, 1200);
    legacy::turn(120, 1277, 1, 1500, 0, 0);
    */
    /*
    legacy::forward(40, 600);
    pros::delay(1000);
    robot::intake.move(0);
    legacy::forward(100, 400);
    */
  }
}