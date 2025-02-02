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

int pause = 0;
void checkIntake(){
  const int timeThreshold = 500;
  const int velocityThreshold = 5;
  while (1){
    if (!pause or !_1028A::auton::autonStop){
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
void _1028A::auton::auton(){
  autonSelect = 1;
  robot::leftMtrs.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
  robot::rightMtrs.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
  if (autonSelect == 1){
    //RedRush
    pros::Task lbTask(_1028A::auton::lbTask);
    pros::Task odomReadTask(odomRead);
    reset = 1;
    robot::stick.set_value(1);
    robot::chassis.moveToPoint(0, 34.25, 2000, {.minSpeed=80}, false);
    armtarget = 100;
    robot::stick.set_value(0);
    pros::delay(150);
    robot::intake.move(127);
    robot::chassis.moveToPoint(0, 26, 2000, {.forwards = false, .minSpeed=80}, false);
    robot::stick.set_value(1);
    robot::chassis.moveToPose(2.200392, 28.575848, 16.070787, 1400, {}, false);
    robot::intake.move(-127);
    pros::delay(20);
    robot::intake.move(0);
    armtarget = 820;
    robot::stick.set_value(0);
    pros::delay(1300);
    robot::chassis.moveToPose(1.446242, 33.879444, 12.158765, 2000, {.minSpeed = 50}, false);
    armtarget = 0;
    robot::chassis.moveToPose(-18.629443, 27.085499, 78.937820, 2000, {.forwards = false, .minSpeed = 40}, false);
    robot::mogo.set_value(1);
    pros::Task intakeTask(_1028A::auton::intakeTask);
    intake = ColorSortBlue;
    pros::delay(150);
    //robot::chassis.moveToPose(9.998731, 31.747925, 100.646179, 2000, {.minSpeed = 50}, false);
    pros::delay(500);
    lbTask.suspend();
    robot::chassis.moveToPose(7.439109, 31.991825, 78.243744, 1300, {.minSpeed = 40}, false);
    pros::delay(500);
    robot::chassis.turnToHeading(-90, 1000, {}, false);
    robot::chassis.moveToPose(23.662140, 24.503874, -74.909660, 1300, {.forwards = false}, false);
    robot::mogo.set_value(0);
    robot::chassis.turnToHeading(151, 1000, {}, false);
    pros::delay(300);
    robot::chassis.moveToPose(10.957269, 40.802235, -181.791260, 1500, {.forwards = false}, false);


    /*
    robot::chassis.moveToPose(37.070717, 3.932603, 170.995651, 3000, {.minSpeed = 50}, false);
    robot::leftMtrs.move(127);
    robot::rightMtrs.move(127);
    pros::delay(400);
    robot::leftMtrs.move(0);
    robot::rightMtrs.move(0);
    pros::delay(300);
    robot::chassis.turnToHeading(286.930664, 2000, {.minSpeed = 30}, false);
    armtarget = 820;
    robot::chassis.moveToPose(-23.273029, 35.008560, 297.188385, 2000, {.minSpeed = 50}, false);
    */
  }
  else if(autonSelect == 3){
    //RedRing
    robot::intake.move(127);
    pros::Task disk(Disk);
    robot::chassis.moveToPoint(1.903392, 38.051418, 2000, {.minSpeed=80}, false);
    pros::delay(200);
    robot::chassis.moveToPose(13.372073, 24.500650, -18.979954, 2000, {.forwards = false, .minSpeed=30}, false);
    robot::mogo.set_value(1);
    pros::delay(200);
    robot::intake.move(127);
    robot::chassis.turnToHeading(-96.865662, 2000, {.minSpeed=30}, false);
    robot::chassis.moveToPose(-9.318869, 31.881939, -90.184494, 2000, {.minSpeed=70}, false);
    robot::chassis.moveToPose(-8.514917, 50.322758, 0, 2000, {.minSpeed=40}, false);
  }
  else if (autonSelect == 4){
    //BlueRingSafe
    pros::Task lbTask(_1028A::auton::lbTask);
    //pros::Task odomReadTask(odomRead);
    pros::Task checkIntakeTask(checkIntake);
    //pros::Task IntakeTask(_1028A::auton::intakeTask);
    armtarget = 500;
    robot::chassis.moveToPoint(0, 1.5, 2000, {.minSpeed = 65}, false);
    pros::delay(200);
    robot::chassis.moveToPose(-22.077450, -27.622503, 84.312889, 2000, {.forwards = false, .minSpeed = 60}, false);
    reset = 1;
    robot::mogo.set_value(1);
    robot::intake.move(127);
    robot::chassis.moveToPose(-16.225252, -49.300167, 200.388962, 2500, {.minSpeed = 60, .earlyExitRange = 2}, false);
    pros::delay(500);
    robot::chassis.moveToPose(-12.379702, -20.952803, 158.844330, 2000, {.forwards = false, .minSpeed = 50}, false);
    robot::chassis.moveToPose(-1.466992, -37.725380, 139.093475, 1500, {.minSpeed = 50}, false);
    pros::delay(2000);
    robot::intake.move(0);
    armtarget = 500;
    robot::chassis.turnToHeading(-90, 1000, {}, false);
    armtarget = 500;
    robot::intake.move(0);
    checkIntakeTask.suspend();
    lbTask.suspend();
    robot::chassis.moveToPose(-29.837498, -26.188953, -45, 2000, {.minSpeed = 10}, false);
  }
  else if (autonSelect == 5){
    //ringRedSafe
    pros::Task lbTask(_1028A::auton::lbTask);
    //pros::Task odomReadTask(odomRead);
    //pros::Task checkIntakeTask(checkIntake);
    //pros::Task IntakeTask(_1028A::auton::intakeTask);
    armtarget = 500;
    robot::chassis.moveToPoint(0, 1.5, 2000, {.minSpeed = 65}, false);
    pros::delay(200);
    robot::chassis.moveToPose(22.077450, -27.622503, -78.312889, 2000, {.forwards = false, .minSpeed = 60}, false);
    robot::mogo.set_value(1);
    reset = 1;
    robot::intake.move(127);
    robot::chassis.moveToPose(18.225252, -51.300167, -204.388962, 2500, {.minSpeed = 60}, false);
    pros::delay(500);
    robot::chassis.moveToPose(14.379702, -25.952803, -158.844330, 2000, {.forwards = false, .minSpeed = 50}, false);
    robot::chassis.moveToPose(-2.505900, -37.667278, -20.307846, 2000, {.minSpeed = 60}, false);
    pros::delay(2000);
    robot::intake.move(0);
    armtarget = 500;
    robot::chassis.turnToHeading(90, 1000, {}, false);
    armtarget = 500;
    robot::LB.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    robot::intake.move(0);
    lbTask.suspend();
    robot::LB.move(0);
    //checkIntakeTask.suspend();
    robot::chassis.moveToPose(29.837498, -26.188953, 45, 2000, {.minSpeed = 10}, false);
  
    /*
    checkIntakeTask.suspend();
     armtarget = 155;
     armOverride = 1;
    robot::chassis.moveToPose(38.839233, -28.592161, -90.065750,  2000, {.forwards = false, .minSpeed = 40}, false);
     robot::LB.move(-30);
    pros::delay(600);
    robot::LB.move(0);
    */
  }
  else if (autonSelect == 6){
    //ringReddouble
    pros::Task lbTask(_1028A::auton::lbTask);
    pros::Task odomReadTask(odomRead);
    pros::Task checkIntakeTask(checkIntake);
    //pros::Task IntakeTask(_1028A::auton::intakeTask);
    armtarget = 500;
    robot::chassis.moveToPoint(0, 1.5, 2000, {.minSpeed = 65}, false);
    pros::delay(200);
    robot::chassis.moveToPose(24.077450, -25.622503, -78.312889, 2000, {.forwards = false, .minSpeed = 60}, false);
    robot::mogo.set_value(1);
    reset = 1;
    robot::intake.move(127);
    robot::chassis.turnToHeading(-175.274719, 600, {.earlyExitRange = 2}, false);
    robot::chassis.moveToPose(11.504578, -48.746521, -137.243942, 2000, {.minSpeed = 50}, false);
    robot::leftMtrs.move(40);
    robot::rightMtrs.move(40);
    pros::delay(600);
    robot::leftMtrs.move(0);
    robot::rightMtrs.move(0);
    pros::delay(300);
    robot::chassis.moveToPose(18.194513, -23.494789, -190.345177, 1800, {.forwards = false, .minSpeed = 50}, false);
    robot::chassis.turnToHeading(-120, 800, {.minSpeed = 30, .earlyExitRange = 2}, false);
    robot::chassis.moveToPose(3.239918, -39.607533, -133.559631, 1500, {.minSpeed = 50}, false);
    pros::delay(800);
    checkIntakeTask.suspend();
    robot::chassis.turnToHeading(-70, 800, {.earlyExitRange = 1.5}, false);
    robot::intake.move(127);
    pros::Task IntakeTask(_1028A::auton::intakeTask);
    intake = ColorSortBlue;
    robot::chassis.moveToPose(-39.771255, -28.904354, -46.379337, 1700, {.minSpeed = 65}, false);
    pros::delay(500);
    robot::leftMtrs.move(127);
    robot::rightMtrs.move(127);
    pros::delay(1000);
    robot::leftMtrs.move(0);
    robot::rightMtrs.move(0);
    

    //robot::chassis.moveToPose(18.225252, -51.300167, -204.388962, 2500, {.minSpeed = 60}, false);

    /*
    pros::delay(500);
    robot::chassis.moveToPose(17.379702, -25.952803, -158.844330, 2000, {.forwards = false, .minSpeed = 60}, false);
    robot::chassis.moveToPose(4.005900, -42.667278, -131.307846, 2000, {.minSpeed = 60}, false);
    pros::delay(2000);
    robot::intake.move(0);
    armtarget = 820;
    robot::chassis.turnToHeading(90, 1000, {}, false);
    robot::intake.move(0);
    checkIntakeTask.suspend();
    robot::chassis.moveToPose(34.837498, -26.188953, 89, 2000, {.minSpeed = 50}, false);
    */
    /*
    checkIntakeTask.suspend();
     armtarget = 155;
     armOverride = 1;
    robot::chassis.moveToPose(38.839233, -28.592161, -90.065750,  2000, {.forwards = false, .minSpeed = 40}, false);
     robot::LB.move(-30);
    pros::delay(600);
    robot::LB.move(0);
    */
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
