#include "1028A/auton.h"
#include "1028A/robot.h"
#include "lemlib/chassis/chassis.hpp"
#include "pros/rtos.hpp"
#include "1028A/logger.h"

double armtarget = 0;
int reset = 0;
int armOverride = 0;
int _1028A::auton::autonStop = 0;
int _1028A::auton::autonSelect = 0;

void _1028A::auton::lbTask() {
  double rotationValue = 0;
  double armP = 0.0005;
  double armD = 0.565;
  double armError = 0;
  double armPrevError = 0;
  double threshold = 0.5;
  double armSpeed = 0;
  double rotationRaw = 0;
  double derivative = 0;
  _1028A::robot::LB.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
  _1028A::robot::LBS.set_reversed(1);
  _1028A::robot::LBS.reset_position();

  while (true) {
    rotationRaw = -_1028A::robot::LBS.get_position();
    rotationValue = rotationRaw / 100;

    armError = armtarget - rotationValue;

    armSpeed = (armError * armP) + (armPrevError * armD);

    // lB.move(armSpeed);

    if (fabs(armError) <= threshold) {
      armSpeed = 0;
    }

    if (reset){
      _1028A::robot::LB.move(127);
      pros::delay(200);
      _1028A::robot::LB.move(-40);
      while (1){
        if (_1028A::robot::LBSLimit.get_value()){
          _1028A::robot::LBS.set_position(0);
          armtarget = 0;
          pros::delay(200);
          _1028A::robot::LB.move(0);
          reset = 0;
          break;
        }
        pros::delay(20);
      }
    }
    else{
      if (!armOverride or !auton::autonStop){
        _1028A::robot::LB.move(armSpeed);
      }
    }

    armPrevError = armError;

    pros::delay(10);
  }
}


enum intakeState{
    INTAKE,
    OUTTAKE,
    STOP,
    ColorSortRed,
    ColorSortBlue
};
intakeState intake = STOP;
void _1028A::auton::intakeTask(){
  bool kickout = false;
   const int timeThreshold = 1000;
  const int velocityThreshold = 5;
  int count = 0;
  int lastCount = 0;
  while (1){
    if (!autonStop){
    if (intake == INTAKE){
      _1028A::robot::optical.set_led_pwm(0);
      _1028A::robot::intake.move(127);
      int initialVelocity = _1028A::robot::intake.get_actual_velocity();

      if (abs(initialVelocity) < velocityThreshold) {
        pros::delay(timeThreshold); 
        int currentVelocity = _1028A::robot::intake.get_actual_velocity();

        if (abs(currentVelocity) < velocityThreshold) {
          _1028A::robot::intake.move(-127);
          pros::delay(300);
          _1028A::robot::intake.move(127);
        }
      }
    }
    else if (intake == OUTTAKE){
      _1028A::robot::optical.set_led_pwm(0);
      _1028A::robot::intake.move(-127);
    }
    else if (intake == STOP){
      _1028A::robot::optical.set_led_pwm(0);
      _1028A::robot::intake.move(0);
    }
    else if (intake == ColorSortBlue){
      if (_1028A::robot::optical.get_hue()>= 200 && _1028A::robot::optical.get_hue() <= 250){
        pros::delay(200);
        kickout = true;
      }
      else{
        kickout = false;
        _1028A::robot::intake.move(127);
        _1028A::robot::optical.set_led_pwm(100);
      }

      if (kickout){
        if (_1028A::robot::distance.get() < 60){
          _1028A::logger::info("KICKOUT");
          pros::delay(80);
          _1028A::robot::intake.move(0);
          pros::delay (300);
          _1028A::robot::intake.move(127);
          kickout = false;
        }
      }
    }
    else if (intake == ColorSortRed){
      if (_1028A::robot::optical.get_hue() <= 10 or _1028A::robot::optical.get_hue() >= 350){
        pros::delay(200);
        kickout = true;
      }
      else{
        kickout = false;
        _1028A::robot::intake.move(127);
        _1028A::robot::optical.set_led_pwm(100);
      }

      if (kickout){
        if (_1028A::robot::distance.get() < 40){
          _1028A::logger::info("KICKOUT");
          pros::delay(80);
          _1028A::robot::intake.move(0);
          pros::delay (300);
          _1028A::robot::intake.move(127);
          kickout = false;
        }
      }
    }
  }
}
}

int Pause = 0;
void checkIntake(){
  const int timeThreshold = 500;
  const int velocityThreshold = 5;
  while (1){
    if (Pause != 1 or !_1028A::auton::autonStop){
    int initialVelocity = _1028A::robot::intake.get_actual_velocity();

      if (abs(initialVelocity) < velocityThreshold) {
        pros::delay(timeThreshold); 
        int currentVelocity = _1028A::robot::intake.get_actual_velocity();

        if (abs(currentVelocity) < velocityThreshold) {
          _1028A::robot::intake.move(-127);
          pros::delay(300);
          _1028A::robot::intake.move(127);
        }
      }
    }
      pros::delay(20);
  }
}

void odomRead (){
    while (1){
        if (1){
            std::string data = "(" + std::to_string(_1028A::robot::chassis.getPose().x) + ", " + std::to_string(_1028A::robot::chassis.getPose().y) + ", " + std::to_string(_1028A::robot::chassis.getPose().theta) + ")";
            _1028A::logger::info(data.c_str());
        }
        pros::delay(400);
    }
}

void Disk(){
  _1028A::robot::intake.move(127);
  while (1){
    if (_1028A::robot::optical.get_proximity() > 253){
               _1028A::robot::intake.move(0);
               break;
            }
    pros::delay(20);
  }
}

void queueDisk(){
  _1028A::robot::intake.move(127);
  while (1){
    if (_1028A::robot::optical.get_proximity() > 253){
               _1028A::robot::intake.move(0);
               break;
            }
    pros::delay(5);
  }
}
void _1028A::auton::auton(){
  autonSelect = 100;
  robot::leftMtrs.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
  robot::rightMtrs.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);

if (autonSelect == 1){
  //redRingRush
  robot::chassis.setPose(0,0,0);
  pros::Task disk(queueDisk);
  robot::chassis.moveToPoint(-13, 40, 2000, {.earlyExitRange = 1.5}, false);
  pros::delay(300);
  robot::chassis.moveToPoint(7, 24, 2000, {.forwards = false}, true);
  robot::chassis.waitUntil(27);
  robot::mogo.set_value(1);
  robot::chassis.waitUntilDone();
  pros::delay(50);
  robot::chassis.turnToPoint(-16, 28, 500, {.earlyExitRange = 2}, false);
  robot::chassis.moveToPoint(-16, 25, 2000, {.earlyExitRange = 2}, true);
  robot::intake.move(127);
  robot::chassis.waitUntilDone();
  robot::chassis.moveToPoint(-26, 35, 2000, {.earlyExitRange = 2}, true);
}
else if (autonSelect == 3){
  //Red Rush
  robot::stick.set_value(1);
  robot::intake.move(127);
  pros::Task queued(queueDisk);
    robot::chassis.moveToPoint(0, 34, 2000, {.minSpeed=80}, false);
    robot::stick.set_value(0);
    robot::chassis.moveToPoint(0, 15, 2000, {.forwards = false}, true);
    robot::chassis.waitUntil(9);
    robot::stick.set_value(1);
    robot::chassis.turnToHeading(-260, 1000, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .earlyExitRange = 4}, false);
    robot::stick.set_value(0);
    robot::chassis.moveToPose(-28, 40.5, -220, 2000, {.forwards = false, .minSpeed=50}, false);
    robot::mogo.set_value(1);
    pros::delay(100);
  robot::intake.move(127);
  robot::chassis.turnToHeading(187, 1000, {.earlyExitRange = 3}, false);
  robot::chassis.moveToPose(-7.5, -5, 102, 2000, {.earlyExitRange = 2}, false);
  robot::leftMtrs.move(50);
  robot::rightMtrs.move(50);
  pros::delay(1700);
  robot::leftMtrs.move(0);
  robot::rightMtrs.move(0);
    

}
else if (autonSelect == 4){
  //Blue Rush
  robot::stick.set_value(1);
  robot::intake.move(127);
  pros::Task queued(queueDisk);
    robot::chassis.moveToPoint(0, 34, 2000, {.minSpeed=80}, false);
    robot::stick.set_value(0);
    robot::chassis.moveToPoint(0, 15, 2000, {.forwards = false}, true);
    robot::chassis.waitUntil(9);
    robot::stick.set_value(1);
    robot::chassis.turnToHeading(260, 1000, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .earlyExitRange = 4}, false);
    robot::stick.set_value(0);
    robot::chassis.moveToPoint(27, 27, 2000, {.forwards = false, .minSpeed=50}, false);
    robot::mogo.set_value(1);
    robot::chassis.turnToHeading(187, 1000, {.earlyExitRange = 3}, false); 
    robot::intake.move(127);
    robot::chassis.moveToPose(-11, -3, 280, 2000, {.minSpeed = 40, .earlyExitRange = 2}, false);
    robot::leftMtrs.move(50);
  robot::rightMtrs.move(50);
  pros::delay(1700);
  robot::leftMtrs.move(0);
  robot::rightMtrs.move(0);

}
else if (autonSelect == 100){
  //skills
  pros::Task odo(odomRead);
  pros::Task checkintake{checkIntake};
  robot::chassis.setPose(0,-3,0);
  pros::Task lbTask(_1028A::auton::lbTask);
  armtarget = 800;
  pros::delay(500);
  reset = 1;
  robot::chassis.moveToPose(30, -5, -108, 1800, {.forwards = false, .minSpeed = 70}, false);
  robot::mogo.set_value(1);
  pros::delay(100);
  robot::chassis.moveToPoint(37, -37, 2000, {.minSpeed = 70}, true);
  robot::intake.move(127);
  robot::chassis.waitUntilDone();
  robot::chassis.moveToPose(54, -74, -200, 1700, {.minSpeed = 70}, false);
  pros::delay(900);
  armtarget = 100;
  robot::chassis.moveToPoint(59, -89, 2000, {.minSpeed = 40}, false);
  checkintake.suspend();
  pros::delay(300);
  robot::chassis.moveToPose(43, -49.5, -196, 2000, {.forwards = false, .minSpeed = 40}, false);
  robot::chassis.turnToHeading(-265, 1000, {}, false);
  robot::intake.move(-127);
  pros::delay(20);
  robot::intake.move(0);
  armtarget = 200;
  pros::delay(200);
  pros::Task queue(queueDisk);
  robot::chassis.moveToPoint(robot::chassis.getPose().x + 20, robot::chassis.getPose().y, 1800, {.maxSpeed = 70, .earlyExitRange = 2}, false);
  armtarget = 580;
  pros::delay(600);
  armtarget = 100;
  pros::delay(600);
  robot::intake.move(127);
  pros::delay(600);
  robot::intake.move(-127);
  pros::delay(20);
  robot::intake.move(0);
  armtarget = 620;
  pros::delay(600);
  robot::chassis.moveToPoint(robot::chassis.getPose().x - 10, robot::chassis.getPose().y, 1800, {.forwards = false, .earlyExitRange = 2}, false);
  robot::chassis.turnToHeading(-353, 500, {}, false);
  robot::intake.move(127);
  checkintake.resume();
  armtarget = 0;
  robot::chassis.moveToPose(58, 14, -354, 2500, {.maxSpeed = 90, .minSpeed = 50}, false);
  pros::delay(400);
  robot::chassis.turnToHeading(90, 800, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .earlyExitRange = 3}, false);
  robot::chassis.moveToPose(67, -15, -194, 2000, {.minSpeed = 50}, false);
  pros::delay(200);
  robot::chassis.turnToHeading(-155, 500, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE}, false);
  robot::chassis.moveToPoint(73, 8, 1000, {.forwards = false, .minSpeed = 50}, false);
  pros::delay(500);
  robot::mogo.set_value(0);
  robot::intake.move(-60);
  robot::chassis.moveToPoint(65, -3, 1000, {.earlyExitRange = 1}, false);
  robot::chassis.turnToHeading(-267, 1000, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE}, false);
  robot::chassis.moveToPose(-20.5, 0, -269, 3500, {.forwards = false, .maxSpeed = 100, .minSpeed = 40, . earlyExitRange = 1}, false);
  robot::mogo.set_value(1);
  pros::delay(100);
  robot::intake.move(127);
  robot::chassis.moveToPoint(-12, 0, 3500, {.minSpeed = 50, .earlyExitRange = 1}, false);
  //checkintake.suspend();
  robot::chassis.moveToPose(-17, -24, -135, 2000, {.minSpeed = 40}, false);
  checkintake.suspend();
  robot::chassis.moveToPose(-40, -69, -164, 2000, {.minSpeed = 70, .earlyExitRange = 2}, false);
  pros::delay(900);
  armtarget = 100;
  robot::chassis.moveToPose(-48, -93, -160.5, 2000, {.minSpeed = 70, .earlyExitRange = 2}, false);
  pros::delay(200);
  robot::chassis.moveToPose(-31, -47.7, -154, 2000, {.forwards = false}, false);
  robot::chassis.turnToHeading(-90, 1000, {}, false);
  robot::intake.move(-127);
  pros::delay(20);
  robot::intake.move(0);
  armtarget = 200;
  pros::delay(200);
  pros::Task Queue(queueDisk);
  robot::chassis.moveToPoint(robot::chassis.getPose().x - 20, robot::chassis.getPose().y, 1800, {.maxSpeed = 70, .earlyExitRange = 2}, false);
  armtarget = 580;
  pros::delay(900);
  armtarget = 100;
  pros::delay(600);
  robot::intake.move(127);
  pros::delay(600);
  robot::intake.move(-127);
  pros::delay(20);
  robot::intake.move(0);
  armtarget = 620;
  pros::delay(900);
  robot::chassis.moveToPoint(robot::chassis.getPose().x + 10, robot::chassis.getPose().y, 1800, {.forwards = false, .earlyExitRange = 2}, false);
  checkintake.resume();
  armtarget = 0;
  robot::chassis.turnToHeading(0, 500, {}, false);
  robot::intake.move(127);
  robot::chassis.moveToPoint(-40, 8, 2500, {.maxSpeed = 90}, false);
  robot::chassis.turnToHeading(-110, 700, {}, false);
  robot::chassis.moveToPose(-55, -12, -176, 1500, {.minSpeed = 60}, false);
  robot::chassis.moveToPoint(-53, 4, 1400, {.forwards = false, .minSpeed = 60, .earlyExitRange = 1.5}, false);
  robot::mogo.set_value(0);
  robot::intake.move(0);
  robot::chassis.turnToHeading(-222, 500, {}, false);
  robot::chassis.moveToPoint(24, -72, 2000, {.minSpeed = 70}, false);
  robot::chassis.turnToHeading(-155, 600, {}, false);
  pros::delay(100);
  robot::chassis.moveToPoint(18, -94, 1400, {.minSpeed = 70, .earlyExitRange = 2}, false); 
  robot::chassis.moveToPoint(68, -106, 2000, {.minSpeed = 70}, false);
  robot::chassis.moveToPose(-6, -102, -270, 2000, {.forwards = false, .minSpeed = 70}, false);
  
}
}
