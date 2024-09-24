#include "1028A/comp/auton.h"
#include "1028A/misc/legacy.h"
#include "1028A/misc/robot.h"
#include "1028A/misc/task.h"
#include "1028A/misc/vars.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"

using namespace okapi;
enum intakeSTS { OFF, Intake, Outtake, Index, FIntake, FOuttake };
intakeSTS intake = OFF;
/*
void intakeJam() {
  while (1) {
    if (_1028A::robot::conveyor.get_target_velocity() > 0 &&
        (_1028A::robot::conveyor.get_actual_velocity()) > 0 &&
        _1028A::robot::conveyor.get_actual_velocity() < 40) {
      _1028A::robot::conveyor.move(-127);
      pros::delay(200);
    } else {

      if (intake == OFF) {
        _1028A::robot::intakeMtrs.move(0);
      } else if (intake == FIntake) {
        _1028A::robot::intake.move(127);
        _1028A::robot::conveyor.move(0);
      } else if (intake == FOuttake) {
        _1028A::robot::intake.move(-127);
        _1028A::robot::conveyor.move(0);
      } else if (intake == Intake) {
        _1028A::robot::intakeMtrs.move(127);
      } else if (intake == Outtake) {
        _1028A::robot::intakeMtrs.move(-127);
      } else if (intake == Index) {
        _1028A::robot::intakeMtrs.move(127);
        _1028A::robot::conveyor.move(-127);
      }
    }

    pros::delay(20);
  }
}

void indexRing() {
  _1028A::robot::intake.move(127);
  _1028A::robot::conveyor.move(127);
  while (1) {
    if (_1028A::robot::ringSenseH.get() < 40) {
      pros::delay(200);
      _1028A::robot::intake.move(127);
      _1028A::robot::conveyor.move(-127);
      pros::delay(1500);
      _1028A::robot::conveyor.move(0);
      pros::delay(500);
      _1028A::robot::intakeMtrs.move(0);
      break;
    }

    pros::delay(10);
  }
}

void grabGoal() {
  while (1) {
    if (fabs(_1028A::robot::leftfront.get_position() * 100) > 250) {
      _1028A::robot::mogo.set_value(1);
      break;
    }
    pros::delay(20);
  }
}
void grabGoal2() {
  while (1) {
    if (fabs(_1028A::robot::leftfront.get_position() * 100) > 350) {
      _1028A::robot::mogo.set_value(1);
      break;
    }
    pros::delay(20);
  }
}

void dropGoal() {
  while (1) {
    if (fabs(_1028A::robot::leftfront.get_position() * 100) >= 700) {
      _1028A::robot::mogo.set_value(0);
      break;
    }
    pros::delay(20);
  }
}

void intakeAsync() {
  while (1) {
    if (fabs(_1028A::robot::leftfront.get_position() * 100) >= 100) {
      _1028A::robot::intakeMtrs.move(127);
      break;
    }
    pros::delay(20);
  }
}

void pulseIntake() {
  while (1) {
    _1028A::robot::intake.move(127);
    pros::delay(400);
    _1028A::robot::intake.move(0);
    pros::delay(20);
  }
}

void deploy() {
  _1028A::robot::conveyor.move(-127);
  pros::delay(800);
  _1028A::robot::conveyor.move(0);
}
ASSET(path_txt);
*/
ASSET(path_txt);
void _1028A::comp::auton() {
  pros::delay(2000);
  robot::chassis.setPose(-58.502, 0.137, 90);
  robot::chassis.follow(path_txt, 30, 100000);
  /*
  // autonSelect = 7;
  //  task::Async JamDetect(intakeJam);
  pros::delay(2000);
  robot::chassis.setPose(-54.879, -63.057, 0);
  robot::chassis.follow(path_txt, 30, 100000);
  // robot::chassis.setPose(0, 0, 0);
  // robot::chassis.moveToPoint(0, 48, 10000);

  if (autonSelect == 1) {
    // BlueL
    robot::conveyor.move(-127);
    pros::delay(1500);
    robot::HGlift.set_value(1);
    pros::delay(300);
    legacy::forward(205, 0, 127, 1, 1000, 0, 0);
    robot::HGlift.set_value(0);
    pros::delay(500);
    legacy::forward(-550, 6, 127, 1, 1000, 0, 0);
    robot::mogo.set_value(1);
    legacy::turn(-14, 127, 1, 1000, 0, 0);
    pros::delay(300);
    robot::Ilift.set_value(1);
    robot::intakeMtrs.move(127);
    legacy::forward(400, 127, 1, 1000, 0, 3.0);
  }

  else if (autonSelect == 2) {
    // RedL
    task::Async goalGrab(grabGoal2);
    legacy::forward(-440, 0, 127, 1, 1000, 0, 0);
    pros::delay(300);
    robot::mogo.set_value(1);
    legacy::turn(63, 127, .5, 1300, 0, 0);
    task::Async intakeTsk(intakeAsync);
    legacy::forward(400, 127, 1, 1000, 0, 0);
    pros::delay(300);
    legacy::forward(-210, 127, 1, 1000, 0, 0);
    pros::delay(300);
    legacy::turn(150, 127, 1, 600, 0, 0);
    pros::delay(200);
    legacy::forward(140, 127, 1, 1000, 0, 0);
    pros::delay(300);
    legacy::forward(-90, 127, 1, 500, 0, 0);
    pros::delay(200);
    legacy::turn(122, 127, 1, 1000, 0, 0);
    pros::delay(200);
    legacy::forward(160, 127, 2, 1000, 0, 0);
    robot::intakeMtrs.move(127);
    pros::delay(500);
    legacy::forward(-200, 127, 1, 1000, 0, 0);
    pros::delay(200);
    legacy::turn(210, 127, 1, 1000, 0, 0);
    pros::delay(200);
    legacy::forward(60, 3000);
    // task::Async indexDisk(indexRing);
    /*
    pros::delay(200);
    legacy::forward(-80, 127, 1, 1000, 0, 0);
    pros::delay(200);
    legacy::turn(314, 127, 1, 800, 0, 0);
    pros::delay(2500);
    robot::HGlift.set_value(1);
    legacy::forward(790, 312.5, 127, 1, 1000, 0, 0);
    robot::HGlift.set_value(0);
    pros::delay(500);
    legacy::forward(-500, 127, 1, 1000, 0, 0);
    legacy::turn(240, 127, 1, 1000, 0, 0);
    legacy::forward(200, 127, 1, 1000, 0, 0);
    */
  /*
  } else if (autonSelect == 3) {
    pros::delay(3000);
    robot::HGlift.set_value(1);
    pros::delay(1000);
    robot::intake.move(127);
    legacy::forward(120, 127, 1, 1000, 0, 0);
    pros::delay(1000);
    robot::HGlift.set_value(0);
    pros::delay(400);
    legacy::forward(-250, 127, 1, 1000, 0, 0);
    legacy::turn(120, 127, 1, 1000, 0, 0);
  } else if (autonSelect == 4) {
    // BlueR
    legacy::forward(-470, 0, 127, 1, 1000, 0, 0);
    pros::delay(300);
    robot::mogo.set_value(1);
    legacy::turn(-63, 127, 1, 1000, 0, 0);
    task::Async intakeTsk(intakeAsync);
    legacy::forward(310, 127, 1, 1000, 0.3, 0);
    pros::delay(300);
    legacy::forward(-210, 127, 1, 1000, 0, 0);
    pros::delay(300);
    legacy::turn(-150, 127, 1, 1000, 0, 0);
    pros::delay(200);
    legacy::forward(140, 127, 1, 1000, 0, 0);
    pros::delay(300);
    legacy::forward(-90, 127, 1, 1000, 0, 0);
    pros::delay(200);
    legacy::turn(-122, 127, 1, 1000, 0, 0);
    pros::delay(200);
    legacy::forward(130, 127, 1, 1000, 0, 0);
    robot::intakeMtrs.move(127);
    pros::delay(400);
    legacy::forward(-350, 127, 1, 1000, 0, 0);
    legacy::turn(90, 127, 1, 1000, 0, 0);
    pros::delay(300);
    legacy::forward(80, 1500);
    /*
    pros::delay(200);
    legacy::forward(-80, 127, 1, 1000, 0, 0);
    pros::delay(200);
    legacy::turn(-314, 127, 1, 1000, 0, 0);
    pros::delay(2500);
    robot::HGlift.set_value(1);
    legacy::forward(720, -313, 127, 1, 1000, 0, 0);
    robot::HGlift.set_value(0);
    pros::delay(500);
    legacy::forward(-400, 127, 1, 1000, 0, 0);
    legacy::turn(-380, 127, 1, 1000, 0, 0);
    */
   /*
  } else if (autonSelect == 7) {
    task::Async goalGrab(grabGoal2);
    legacy::forward(-370, 0, 127, 1, 700, 0, 0);
    pros::delay(100);
    robot::mogo.set_value(1);
    legacy::turn(95, 127, .5, 800, 0, 0);
    robot::intakeMtrs.move(127);
    pros::delay(100);
    legacy::forward(280, 127, 1, 800, 0, 0);
    pros::delay(100);
    legacy::turn(58, 127, 1, 800, 0, 0);
    pros::delay(100);
    legacy::forward(220, 127, 1, 500, 0, 0);
    // robot::conveyor.move(0);
    legacy::forward(-500, -180, 127, 1, 1000, 0, 0);
    robot::intakeMtrs.move(127);
    legacy::turn(48, 127, 1, 1000, 0, 0);
    pros::delay(300);
    legacy::forward(300, 127, 1, 1000, 0, 0);
  } else if (autonSelect == 8) {
    task::Async goalGrab(grabGoal2);
    legacy::forward(-370, 0, 127, 1, 700, 0, 0);
    pros::delay(100);
    robot::mogo.set_value(1);
    legacy::turn(-95, 127, .5, 800, 0, 0);
    robot::intakeMtrs.move(127);
    pros::delay(100);
    legacy::forward(280, 127, 1, 800, 0, 0);
    pros::delay(100);
    legacy::turn(-58, 127, 1, 800, 0, 0);
    pros::delay(100);
    legacy::forward(180, 127, 1, 500, 0, 0);
    legacy::forward(-500, -180, 127, 1, 1000, 0, 0);
    robot::intakeMtrs.move(127);
    legacy::turn(-48, 127, 1, 1000, 0, 0);
    pros::delay(300);
    legacy::forward(300, 127, 1, 1000, 0, 0);
    /*
    legacy::turn(-40, 127, 1, 1000, 0, 0);
    task::Async indexDisk(indexRing);
    pros::delay(100);
    legacy::forward(250, 127, 1, 1000, 0, 0);
    pros::delay(300);
    legacy::turn(68, 127, 1, 1000, 0, 0);
    pros::delay(2800);
    robot::HGlift.set_value(1);
    pros::delay(300);
    legacy::forward(600, 68, 127, 1, 1000, 0, 0);
    robot::HGlift.set_value(0);
    pros::delay(400);
    legacy::forward(-200, 127, 1, 1000, 0, 0);
    */
   /*
  } else if (autonSelect == 16) {
    // Skills
    robot::HGlift.set_value(1);
    pros::delay(500);
    legacy::forward(100, 40, 1, 4000, 0, 0);
    robot::HGlift.set_value(0);
    pros::delay(300);
    legacy::forward(-140, 127, 1, 1000, 0, 0);
    legacy::turn(87, 127, 1, 1000, 0, 0);
    pros::delay(300);
    task::Async goalGrab(grabGoal2);
    legacy::forward(-300, 86, 127, 1, 1000, 0, 0);
    robot::mogo.set_value(1);
    pros::delay(300);
    legacy::turn(172, 127, 1, 1000, 0, 0);
    robot::intakeMtrs.move(127);
    pros::delay(300);
    legacy::forward(300, 127, 1, 1000, 0, 0);
    pros::delay(300);
    legacy::turn(230, 127, 1, 1000, 0, 0);
    pros::delay(300);
    legacy::forward(400, 127, 1, 1000, 0, 1.0);
    pros::delay(300);
    legacy::forward(-100, 127, 1, 1000, 0, 0);
    legacy::turn(330, 127, 1, 1000, 0, 0);
    legacy::forward(600, 360, 90, 1, 4000, 0, 0);

    /*
    legacy::turn(87, 127, 1, 1000, 0, 0);
    pros::delay(500);
    legacy::forward(600, 40, 1, 4000, 0, 0);
    pros::delay(200);
    legacy::turn(-35, 127, 1, 1000, 0, 0);
    pros::delay(200);
    legacy::forward(300, 80, 1, 1000, 0, 0);
    pros::delay(200);
    legacy::turn(10, 127, 1, 1000, 0, 0);
    pros::delay(200);
    legacy::forward(130, 80, 1, 1000, 0, 0);
    legacy::turn(-35, 127, 1, 1000, 0, 0);
    */
  }