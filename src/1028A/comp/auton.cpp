#include "1028A/comp/auton.h"
#include "1028A/misc/legacy.h"
#include "1028A/misc/robot.h"
#include "1028A/misc/task.h"
#include "1028A/misc/logger.h"
#include "1028A/misc/vars.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"

using namespace okapi;
enum intakeSTS { OFF, Intake, Outtake, Index, FIntake, FOuttake };
intakeSTS intake = OFF;

void grabOnMove(){
  while (1){
    if (fabs(_1028A::robot::leftfront.get_position() * 100) > 400){
      _1028A::robot::mogo.set_value(1);
      break;
    }

    pros::delay(20);
  }
}

void grabOnMove2(){
  while (1){
    if (fabs(_1028A::robot::leftfront.get_position() * 100) > 950){
      _1028A::robot::mogo.set_value(1);
      break;
    }

    pros::delay(20);
  }
}

void grabwhileback(){
  while (1){
    if (fabs(_1028A::robot::leftfront.get_position() * 100) > 250){
      _1028A::robot::mogo.set_value(1);
      break;
    }

    pros::delay(20);
  }
}

void checkIntakeMotor() {
  const int timeThreshold = 1000;
  const int velocityThreshold = 5;

  while (true) {
    int initialVelocity = _1028A::robot::intakeL.get_actual_velocity();

    if (abs(initialVelocity) < velocityThreshold) {
      pros::delay(timeThreshold); 
      int currentVelocity = _1028A::robot::intakeL.get_actual_velocity();

      if (abs(currentVelocity) < velocityThreshold) {
        _1028A::robot::intakeMtrs.move(-127);
        pros::delay(300);
        _1028A::robot::intakeMtrs.move(127);
      }
    }

    pros::delay(200);
  }
}

void liftArms(){
  _1028A::robot::intakeMtrs.move(-100);
  pros::delay(520);
  _1028A::robot::intakeMtrs.move(0);
}

void dropGoal(){
  while (1){
    if (fabs(_1028A::robot::leftfront.get_position() * 100) > 650){
      _1028A::robot::mogo.set_value(0);
      _1028A::robot::intakeMtrs.move(127);
      break;
    }

    pros::delay(20);
  }
}

void startIntake(){
  while (1){
    if (fabs(_1028A::robot::leftfront.get_position() * 100) > 300){
      _1028A::robot::intakeMtrs.move(127);
      break;
    }
  }
}

void Macro(){
    while (1){
        if (_1028A::robot::ringL.get()<20){
          _1028A::robot::intakeMtrs.move(60);
          while (1){
            if (_1028A::robot::ring.get()<25){
              _1028A::robot::intakeMtrs.move(0);
              pros::delay(300);
              _1028A::robot::intakeMtrs.move(-80);
              pros::delay(200);
              _1028A::robot::intakeMtrs.move(0);
              macroStart = 0;
            }
            else if (!macroStart){
              break;
            }
            pros::delay(10);
          }
        }
        pros::delay(20);
  }
}
void _1028A::comp::auton() {
  autonSelect =12;
  if(autonSelect == 1){
  //RedNegWP
  task::Async grabGoal(grabOnMove);
  legacy::forward(-460, 127, 4, 1100,-.3,0);
  pros::delay(200);
  legacy::turn(85, 127, 1, 1000, 0, 0);
  robot::intakeMtrs.move(127);
  task::Async checkIntake(checkIntakeMotor);
  legacy::forward(300, 85, 80, 1, 1000, 0, 0);
  legacy::forward(150, 65, 60, 1, 2000,0,0);
  pros::delay(500);
  legacy::forward(-380, 100, 127, 1, 1000,0,0);
  legacy::turn(50, 127,1, 1000,0,0);
  pros::delay(200);
  legacy::forward(350, 127, 1, 1000,0,0);
  pros::delay(800);
  legacy::turn(-73, 127, 1, 1000,0,0);
  pros::delay(300);
  robot::intakeMtrs.move(0);
  task::Async liftARMS(liftArms);
  legacy::forward(780, -71, 80, 1, 2500,0,0);
  robot::intakeMtrs.move(127);
  pros::delay(500);
  legacy::forward(-500, -71, 127,  1, 1000,0,0);
  legacy::turn(-155, 127, 1, 1000,0,0);
  pros::delay(200);
  legacy::forward(200, 80, 1, 1000,0,0);
  checkIntake.stopTask();
  _1028A::robot::intakeMtrs.move(-127);
  pros::delay(900);
  _1028A::robot::intakeMtrs.move(0);
  }
  else if(autonSelect == 2){
  //RedNegElim
  task::Async grabGoal(grabOnMove);
  legacy::forward(-460, 127, 4, 1100,-.3,0);
  pros::delay(200);
  legacy::turn(88, 127, 1, 1000, 0, 0);
  robot::intakeMtrs.move(127);
  task::Async checkIntake(checkIntakeMotor);
  legacy::forward(290, 85, 80, 1, 1000, 0, 0);
  legacy::forward(150, 65, 60, 1, 2000,0,0);
  pros::delay(800);
  legacy::forward(-380, 100, 127, 1, 1000,0,0);
  legacy::turn(50, 127,1, 1000,0,0);
  pros::delay(200);
  legacy::forward(350, 127, 1, 1000,0,0);
  pros::delay(200);
  legacy::turn(-90, 127, 1, 1000,0,0);  
  pros::delay(1000);
  robot::intakeMtrs.move(0);
  checkIntake.stopTask();
  legacy::forward(900, -105, 127, 3, 3000, 0,0);
  legacy::forward(700, -90, 127, 1, 4000,0,0);
  /*
  pros::delay(800);
  legacy::turn(58, 127, 1, 1000,0,0);
  robot::intakeMtrs.move(0);
  legacy::forward(300, 127, 3, 1000,0,0);
  legacy::forward(900, -110, 127, 1, 4000,0,0);
  */
  }
  else if(autonSelect == 3){
  //BlueNegWP
  task::Async grabGoal(grabOnMove);
  legacy::forward(-460, 127, 4, 1100,-.3,0);
  pros::delay(200);
  legacy::turn(-85, 127, 1, 1000, 0, 0);
  robot::intakeMtrs.move(127);
  task::Async checkIntake(checkIntakeMotor);
  legacy::forward(300, -85, 80, 1, 1000, 0, 0);
  legacy::forward(150, -65, 60, 1, 2000,0,0);
  pros::delay(500);
  legacy::forward(-380, -100, 127, 1, 1000,0,0);
  legacy::turn(-50, 127,1, 1000,0,0);
  pros::delay(200);
  legacy::forward(350, 127, 1, 1000,0,0);
  pros::delay(800);
  legacy::turn(73, 127, 1, 1000,0,0);
  pros::delay(300);
  robot::intakeMtrs.move(0);
  task::Async liftARMS(liftArms);
  legacy::forward(780, 71, 80, 1, 2500,0,0);
  robot::intakeMtrs.move(127);
  pros::delay(500);
  legacy::forward(-500, 71, 127,  1, 1000,0,0);
  legacy::turn(155, 127, 1, 1000,0,0);
  pros::delay(200);
  legacy::forward(200, 80, 1, 1000,0,0);
  checkIntake.stopTask();
  _1028A::robot::intakeMtrs.move(-127);
  pros::delay(900);
  _1028A::robot::intakeMtrs.move(0);
  }
  else if(autonSelect == 4){
  //BlueNegElim
  task::Async grabGoal(grabOnMove);
  legacy::forward(-460, 127, 4, 1100,-.3,0);
  pros::delay(200);
  legacy::turn(-88, 127, 1, 1000, 0, 0);
  robot::intakeMtrs.move(127);
  task::Async checkIntake(checkIntakeMotor);
  legacy::forward(290, -85, 80, 1, 1000, 0, 0);
  legacy::forward(150, -65, 60, 1, 2000,0,0);
  pros::delay(800);
  legacy::forward(-380, -100, 127, 1, 1000,0,0);
  legacy::turn(-50, 127,1, 1000,0,0);
  pros::delay(200);
  legacy::forward(350, 127, 1, 1000,0,0);
  pros::delay(200);
  legacy::turn(90, 127, 1, 1000,0,0);  
  pros::delay(1000);
  robot::intakeMtrs.move(0);
  legacy::forward(900, 105, 127, 3, 3000, 0,0);
  robot::intakeMtrs.move(127);
  legacy::forward(600, 90, 127, 1, 4000,0,0);
  /*
  pros::delay(800);
  legacy::turn(58, 127, 1, 1000,0,0);
  robot::intakeMtrs.move(0);
  legacy::forward(300, 127, 3, 1000,0,0);
  legacy::forward(900, -110, 127, 1, 4000,0,0);
  */
  }
  else if (autonSelect == 5){
   
  }else if (autonSelect == 12){
    //Skills
    task::Async checkIntake(checkIntakeMotor);
    robot::intakeMtrs.move(127);
    pros::delay(1000);
    robot::intakeMtrs.move(0);
    legacy::forward(195, 127, 1, 1000,0,0);
    pros::delay(200);
    legacy::turn(90, 127, 1, 1000,0,0);
    pros::delay(200);
    task::Async grabwhileBack(grabwhileback);
    legacy::forward(-310, 127, 1, 1000,0,0);
    pros::delay(200);
    legacy::turn(0, 127, 1, 1000,0,0);
    pros::delay(200);
    robot::intakeMtrs.move(127);
    legacy::forward(300, 127, 1, 1000,0,0);
    pros::delay(500);
    robot::intakeMtrs.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
    robot::intakeMtrs.move(0);
    legacy::turn(45, 127, 1, 1000,0,0);
    task::Async StartIntake(startIntake);
    pros::delay(200);
    legacy::forward(500, 45, 127, 1, 1000,0,0);
    robot::intakeMtrs.move(127);
    pros::delay(800);
    robot::intakeMtrs.move(0);
    legacy::forward(-500, 45, 127, 1, 1000,0,0);
    robot::intakeMtrs.move(127);
    StartIntake.stopTask();
    checkIntake.stopTask();
    task::Async diskIndex (Macro);
    pros::delay(200);
    legacy::turn(-60, 127, 1, 1000,0,0);
    pros::delay(200);
    legacy::forward(400, 127, 1, 1000,0,0);
    /*
    pros::delay(200);
    legacy::turn(-55, 127, 1, 1000,0,0);
    pros::delay(200);
    legacy::forward(400, 127, 1, 1000,0,0);
    pros::delay(900);
    legacy::turn(-195, 127, 1, 1000,0,0);
    pros::delay(200);
    legacy::forward(180, 127, 1, 1000,0,0);
    pros::delay(200);
    legacy::turn(-180, 127,1, 1000,0,0);
    pros::delay(200);
    legacy::forward(480, 40, 1, 5000,0,0);
    pros::delay(500);
    legacy::forward(-60, 127, 1, 1000,0,0);
    legacy::turn(-90, 127, 1, 1000,0,0);
    pros::delay(200);
    legacy::forward(200, 127, 1, 1000,0,0);
    pros::delay(500);
    legacy::forward(-180, 127, 1, 1000,0,0);
    pros::delay(200);
    legacy::turn(45, 127, 1, 1000,0,0);
    pros::delay(200);
    legacy::forward(-150, 127, 1, 1000,0,0);
    pros::delay(200);
    robot::mogo.set_value(0);
     robot::intakeMtrs.move(-20);
    pros::delay(200);
    robot::intakeMtrs.move(0);
    legacy::turn(60, 127,1 ,1000,0,0);
    legacy::forward(190, 127, 1, 1000,0,0);
    */

    /*
    pros::delay(200);
    legacy::turn(-88, 127, 1, 1000,0,0);
    pros::delay(300);
    task::Async grabGoal2(grabOnMove2);
    legacy::forward(-990, -91, 127, 1, 1500,0,0);
    pros::delay(200);
    legacy::turn(87, 127, 1, 1500,0,0);
    pros::delay(300);
    robot::intakeMtrs.move(127);
    pros::delay(300);
    legacy::forward(400, 80, 1, 1000,0,0);
    pros::delay(500);
    legacy::turn(0, 127, 1, 1500,0,0);
    legacy::forward(-200, 127, 1, 1000,0,0);
    pros::delay(200);
    robot::mogo.set_value(0);
    legacy::forward(400, 127, 1, 1000,0,0);
    */

    /*
    pros::delay(200);
    legacy::forward(210, 127, 1, 1000,0,0);
    pros::delay(200);
    legacy::turn(177, 127,1, 1000,0,0);
    pros::delay(200);
    legacy::forward(480, 40, 1, 5000,0,0);
    pros::delay(500);
    legacy::forward(-130, 127, 1, 1000,0,0);
    legacy::turn(90, 127, 1, 1000,0,0);
    pros::delay(200);
    legacy::forward(180, 127, 1, 1000,0,0);
    pros::delay(500);
    legacy::forward(-180, 127, 1, 1000,0,0);
    pros::delay(200);
    legacy::turn(-45, 127, 1, 1000,0,0);
    pros::delay(200);
    legacy::forward(-300, 127, 1, 1000,0,0);
    */
  }
  }