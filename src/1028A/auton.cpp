#include "1028A/auton.h"
#include "1028A/robot.h"
#include "lemlib/chassis/chassis.hpp"
#include "pros/rtos.hpp"
#include "1028A/logger.h"

double armtarget = 0;
int reset = 0;
int _1028A::auton::autonStop = 0;
int armOverride = 0;
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

  _1028A::robot::LBS.set_reversed(4);

  while (true) {
    if (!autonStop){
    rotationRaw = -_1028A::robot::LBS.get_position();
    rotationValue = rotationRaw / 100;

    armError = armtarget - rotationValue;
    armSpeed = (armError * armP) + (armPrevError * armD);

    // lB.move(armSpeed);

    if (fabs(armError) <= threshold) {
      armSpeed = 0;
    }
    if (reset){
      _1028A::robot::LB.move(-40);
      while (1){
        if (_1028A::robot::LBSLimit.get_value()){
          armtarget = 0;
          _1028A::robot::LBS.reset_position();
          pros::delay(100);
          _1028A::robot::LB.move(0);
          reset = 0;
          break;
        }
      }
    }
    else if (armOverride == 1){
      pros::delay(100);
    }
    else{
      _1028A::robot::LB.move(armSpeed);
    }

    armPrevError = armError;

    pros::delay(10);
  }
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
  while (1){
    if (!autonStop){
    if (intake == INTAKE){
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
      _1028A::robot::intake.move(-127);
    }
    else if (intake == STOP){
      _1028A::robot::intake.move(0);
    }
    else if (intake == ColorSortBlue){
     
      if (_1028A::robot::optical.get_hue()> 140){
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
  }
}
}

int pause = 0;
void checkIntake(){
  const int timeThreshold = 500;
  const int velocityThreshold = 5;
  while (1){
    if (!pause){
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


void _1028A::auton::auton(){
  autonSelect = 1;
  if (autonSelect == 1){
    pros::Task lbTask(_1028A::auton::lbTask);
    pros::Task odomReadTask(odomRead);
    pros::Task checkIntakeTask(checkIntake);
    //pros::Task intakeTask(_1028A::auton::intakeTask);
     armtarget = 400;

    robot::chassis.moveToPoint(0, 1.5, 2000, {.minSpeed = 40}, false);
    pros::delay(200);
    robot::chassis.moveToPose(22.077450, -27.622503, -90.312889, 2000, {.forwards = false, .minSpeed = 60}, false);
    robot::mogo.set_value(1);
    reset = 1;
    robot::chassis.moveToPose(7.925098, -39.530811, -132.770615, 2000, {.minSpeed = 40}, false);
    checkIntakeTask.suspend();
    pros::delay(2000);
    robot::intake.move(0);
    robot::chassis.moveToPose(-16.820389, -39.958199, -300.666565, 2000, {.forwards = false, .minSpeed = 40}, false);
    //robot::chassis.moveToPose(-23.514460, -21.155567, -256.111389, 2000, {.forwards = false, .minSpeed = 40}, false);
    robot::mogo.set_value(0);
    robot::chassis.moveToPose(3.995605, -45.530689, -210.416855, 2000, {.minSpeed = 40}, false);
    robot::chassis.turnToHeading(-390, 2000, {}, false);
  }
  else if (autonSelect == 2){
    pros::Task lbTask(_1028A::auton::lbTask);
    pros::Task odomReadTask(odomRead);
    pros::Task checkIntakeTask(checkIntake);
    //pros::Task IntakeTask(_1028A::auton::intakeTask);
   armtarget = 500;
    robot::chassis.moveToPoint(0, 1.5, 2000, {.minSpeed = 40}, false);
    pros::delay(200);
    robot::chassis.moveToPose(-22.077450, -27.622503, 78.312889, 2000, {.forwards = false, .minSpeed = 40}, false);
    robot::mogo.set_value(1);
    reset = 1;
    robot::intake.move(127);
    robot::chassis.moveToPose(-18.225252, -52.300167, 204.388962, 2500, {.minSpeed = 40}, false);
    pros::delay(500);
    robot::chassis.moveToPose(-17.379702, -25.952803, 158.844330, 2000, {.forwards = false, .minSpeed = 40}, false);
    robot::chassis.moveToPose(-4.005900, -42.667278, 131.307846, 2000, {.minSpeed = 40}, false);
    pros::delay(1000);
    checkIntakeTask.suspend();
    robot::intake.move(0);
    armtarget = 165;
    armOverride = 1;
    robot::chassis.moveToPose(-38.839233, -28.592161,  90.065750,  2000, {.forwards = false, .minSpeed = 40}, false);
    robot::LB.move(-30);
    pros::delay(600);
    robot::LB.move(0);

    /*
    robot::chassis.moveToPose(-27.376768, -27.053391, 72.227409, 2000, {.forwards = false, .minSpeed = 70}, false);
    reset = 1;
    robot::mogo.set_value(1);
    robot::intake.move(127);
    robot::chassis.moveToPose(-21.545387, -47.756348, 184.505676, 2000, {.minSpeed = 30}, false);
    pros::delay(400);
    robot::chassis.moveToPose(-20.628834, -30.791895, 182.407166, 2000, {.forwards = false, .minSpeed = 40}, false);
    robot::chassis.moveToPose(-12.914774, -40.164303, 133.656204, 2000, {.minSpeed = 40}, false);
    pros::delay(800);
    robot::intake.move(0);
    robot::chassis.moveToPose(-37.396881, -25.883064, 116.157562, 3000, {.forwards = false, .maxSpeed = 60, .minSpeed = 20}, false);
    armtarget = 120;
    */
  }
  else if (autonSelect == 3){
    pros::Task lbTask(_1028A::auton::lbTask);
    pros::Task odomReadTask(odomRead);
    pros::Task checkIntakeTask(checkIntake);
    //pros::Task IntakeTask(_1028A::auton::intakeTask);
    armtarget = 500;
    robot::chassis.moveToPoint(0, 1.5, 2000, {.minSpeed = 40}, false);
    pros::delay(200);
    robot::chassis.moveToPose(22.077450, -27.622503, -78.312889, 2000, {.forwards = false, .minSpeed = 40}, false);
    robot::mogo.set_value(1);
    reset = 1;
    robot::intake.move(127);
    robot::chassis.moveToPose(18.225252, -51.300167, -204.388962, 2500, {.minSpeed = 40}, false);
    pros::delay(500);
    robot::chassis.moveToPose(17.379702, -25.952803, -158.844330, 2000, {.forwards = false, .minSpeed = 40}, false);
    robot::chassis.moveToPose(4.005900, -42.667278, -131.307846, 2000, {.minSpeed = 40}, false);
    pros::delay(2000);
    robot::intake.move(0);
    checkIntakeTask.suspend();
     armtarget = 155;
     armOverride = 1;
    robot::chassis.moveToPose(38.839233, -28.592161, -90.065750,  2000, {.forwards = false, .minSpeed = 40}, false);
     robot::LB.move(-30);
    pros::delay(600);
    robot::LB.move(0);
  }
  else if (autonSelect == 4){
    pros::Task lbTask(_1028A::auton::lbTask);
    pros::Task odomReadTask(odomRead);
    pros::Task checkIntakeTask(checkIntake);
    //pros::Task intakeTask(_1028A::auton::intakeTask);
     armtarget = 400;

    robot::chassis.moveToPoint(0, 1.5, 2000, {.minSpeed = 40}, false);
    pros::delay(200);
    robot::chassis.moveToPose(-22.077450, -27.622503, 90.312889, 2000, {.forwards = false, .minSpeed = 60}, false);
    robot::mogo.set_value(1);
    reset = 1;
    robot::chassis.turnToHeading(175, 1000, {}, false);
    robot::intake.move(127);
    robot::chassis.moveToPose(-10.782574, -52.131386, 140.292114, 2000, {.minSpeed = 60}, false);
    pros::delay(1000);


  }
  else if (autonSelect == 100){
    pros::Task lbTask(_1028A::auton::lbTask);
    pros::Task odomReadTask(odomRead);
    pros::Task checkIntakeTask(checkIntake);
    armtarget = 600;
    pros::delay(600);
    robot::chassis.moveToPose(29.746258, -4.605053, -107.944908, 2000, {.forwards = false, .minSpeed = 50}, false);
    reset = 1;
    robot::mogo.set_value(1);
    //intake = INTAKE;
    robot::intake.move(127);
    robot::chassis.moveToPoint(30.961267, -28.328779, 2000, {.minSpeed = 40}, false);
    robot::chassis.moveToPose(51.848095, -71.922630, -199.587311, 2000, {.minSpeed = 40}, false);
    robot::chassis.moveToPose(41.914261, -50.042350, -201.850739, 2000, {.forwards = false, .minSpeed = 40}, false);
    robot::chassis.turnToHeading(-270, 2000, {}, false);
    pause = 1;
    armtarget = 110;
    robot::chassis.moveToPoint(robot::chassis.getPose().x + 10, robot::chassis.getPose().y, 2000, {.minSpeed = 10}, false);
    pros::delay(1000);
    //intake = STOP;
    robot::intake.move(0);
    robot::chassis.moveToPoint(robot::chassis.getPose().x + 10, robot::chassis.getPose().y, 2000, {.minSpeed = 40}, false);
    robot::intake.move(-127);
    pros::delay(30);
    robot::intake.move(0);
    armtarget = 600;
    //intake = OUTTAKE;
    pros::delay(500);
    armtarget = 0;
    pros::delay(300);
    pause = 0;
    robot::chassis.moveToPoint(robot::chassis.getPose().x - 15, robot::chassis.getPose().y, 2000, {.forwards = false, .minSpeed = 40}, false);
    //intake = INTAKE;
    robot::intake.move(127);
    robot::chassis.turnToPoint(49.997654, -1.604250, 2000, {}, false);
    robot::chassis.moveToPose(49.997654, 4, -355, 2500, {.maxSpeed = 50, .minSpeed = 30}, false);
    robot::chassis.moveToPose(66.179131, -15.496865, -199.154053, 2000, {.maxSpeed = 50, .minSpeed = 30}, false);
    robot::chassis.moveToPose(67.841583, 10.705062, -149.630829, 2000, {.forwards = false, .minSpeed = 40}, false);
    robot::mogo.set_value(0);
    robot::intake.move(-60);
    robot::chassis.moveToPose(57.041306, -5, -138.306213, 2000, {.minSpeed = 40}, false);
    robot::chassis.turnToHeading(-271, 2000, {}, false);
    robot::chassis.moveToPose(-25.703808, -5, -271.5, 3000, {.forwards = false, .maxSpeed = 90, .minSpeed = 40,}, false);
    robot::mogo.set_value(1);
    robot::intake.move(127);
    robot::chassis.turnToHeading(-180, 1000, {.earlyExitRange = 1}, false);
    robot::chassis.moveToPose(-20.357996, -30.326389, -148.028763, 2000, {.minSpeed = 30}, false);
    robot::chassis.moveToPose(-45.344822, -75.500053, -151.155777, 2000, {.minSpeed = 40}, false);
    robot::chassis.moveToPose(-31.312790, -51.761909, -150.793716, 2000, {.forwards = false, .minSpeed = 40}, false);
    robot::chassis.turnToHeading(-90, 2000, {}, false);
    pause = 1;
    armtarget = 110;
    robot::chassis.moveToPoint(robot::chassis.getPose().x - 10, robot::chassis.getPose().y, 2000, {.minSpeed = 10}, false);
     pros::delay(1000);
    robot::intake.move(0);
    robot::chassis.moveToPoint(robot::chassis.getPose().x - 10, robot::chassis.getPose().y, 2000, {.minSpeed = 40}, false);
    robot::intake.move(-127);
    pros::delay(30);
    robot::intake.move(0);
    armtarget = 600;
    pros::delay(500);
    armtarget = 0;
    pros::delay(300);
    pause = 0;
    robot::chassis.moveToPoint(robot::chassis.getPose().x + 13.5, robot::chassis.getPose().y, 2000, {.forwards = false, .minSpeed = 40}, false);
    robot::chassis.turnToHeading(-4, 1000, {}, false);
    robot::chassis.moveToPose(-40.036289, 1.114113, -4.425088, 3000, {.minSpeed = 30}, false);
    robot::chassis.moveToPose(-49.768208, -13.869328, -179.322510, 2000, {.maxSpeed = 60, .minSpeed = 40}, false);
    robot::chassis.moveToPose(-54.343880, 6.450192, -203.242996, 2000, {.forwards = false, .minSpeed = 40}, false);
    pros::delay(1000);
    robot::mogo.set_value(0);
    robot::intake.move(-60);
    pros::delay(100);
    robot::intake.move(0);
    robot::chassis.moveToPose(-17.686520, -79.349236, -215.876007, 3000, {.minSpeed = 50}, false);
    robot::intake.move(127);
    robot::chassis.turnToHeading(-73, 2000, {}, false);
    robot::chassis.moveToPose(19.260771, -108.481918, -49.885487, 2000, {.forwards = false, .minSpeed = 40}, false);
    robot::mogo.set_value(1);
    robot::chassis.moveToPose(-45.658627, -105.677155, -93.913025, 2000, {.minSpeed = 40}, false);
    robot::chassis.turnToHeading(30, 2000, {}, false);
    robot::chassis.moveToPoint(robot::chassis.getPose().x -3, robot::chassis.getPose().y - 7, 2000, {.forwards = false, .minSpeed = 40}, false);
    robot::mogo.set_value(0);
    robot::intake.move(0);
    robot::chassis.turnToHeading(90, 2000, {}, false);
    robot::chassis.moveToPose(60.345459, -111.519676, 97.322800, 5000, {.minSpeed = 60}, false);
  }
}
