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

void intakeFunny(){
  _1028A::robot::intakeMtrs.move(127);
  pros::delay(200);
  _1028A::robot::intakeMtrs.move(0);
  
}

void grabwhileback(){
  while (1){
    if (fabs(_1028A::robot::leftfront.get_position() * 100) > 240){
      _1028A::robot::mogo.set_value(1);
      break;
    }

    pros::delay(20);
  }
}

void checkIntakeMotor() {
  const int timeThreshold = 600;
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
  _1028A::robot::intakeMtrs.move(-127);
  pros::delay(900);
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
    _1028A::robot::intakeMtrs.move(127);
      while (1){
        if (_1028A::robot::ringL.get() < 40){
          pros::delay(30);
          _1028A::robot::intakeMtrs.move(70);
          while (1){
            if (_1028A::robot::ring.get() < 40){
              _1028A::robot::intakeMtrs.move(0);
              break;
            }
            pros::delay(5);
          }
          break;
        }
        pros::delay(5);
      }
}

void grabonBack2(){
  _1028A::robot::leftfront.tare_position();
  while (1){
    if (fabs(_1028A::robot::leftfront.get_position() * 100) > 840){
      _1028A::robot::mogo.set_value(1);
      break;
    }
    int pos  = _1028A::robot::leftfront.get_position() * 100;
    _1028A::robot::master.print(1,1,"pos: %i", pos);
    pros::delay(20);
  }
}

void grabonBack3(){
  _1028A::robot::leftfront.tare_position();
  while (1){
    if (fabs(_1028A::robot::leftfront.get_position() * 100) > 400){
      _1028A::robot::mogo.set_value(1);
      break;
    }
    int pos  = _1028A::robot::leftfront.get_position() * 100;
    _1028A::robot::master.print(1,1,"pos: %i", pos);
    pros::delay(20);
  }
}
void armsHalfUp(){
  _1028A::robot::intakeMtrs.move(-127);
  pros::delay(250);
  _1028A::robot::intakeMtrs.move(0);
}

void armsDownonFWD(){
  while (1){
    if (fabs(_1028A::robot::leftfront.get_position() * 100) > 180){
      _1028A::task::Async intakeFUNNY(intakeFunny);
      break;
    }
    pros::delay(5);
  }
}

void intakeToBottom(){
  _1028A::robot::intakeMtrs.move(127);
  while (1){
    if (_1028A::robot::ringL.get() < 40){
      _1028A::robot::intakeMtrs.move(0);
      break;
    }
    pros::delay(5);
  }
}

void goalGrabonBack5(){
  while (1){
    if (fabs(_1028A::robot::leftfront.get_position() * 100) > 190){
      _1028A::robot::mogo.set_value(1);
      _1028A::robot::intakeMtrs.move(127);
      break;
    }
    pros::delay(5);
  }
}

void goalGrabonBack9(){
  _1028A::robot::leftfront.tare_position();
  while (1){
    if (fabs(_1028A::robot::leftfront.get_position() * 100) > 480){
      _1028A::robot::mogo.set_value(1);
      break;
    }
    pros::delay(5);
  }
}

void dropOnWay(){
  while (1){
    if (fabs(_1028A::robot::leftfront.get_position() * 100) > 300){
      _1028A::robot::mogo.set_value(0);
      break;
    }
    pros::delay(5);
  }
}

void stickuponturn(){
  while (1){
    if (fabs(_1028A::robot::inertialReg.get_rotation()) <= 20){
      _1028A::robot::stick.set_value(0);
      break;
    }
    pros::delay(5);
  }
}
void _1028A::comp::auton() {
  autonSelect = 11;
  if(autonSelect == 1){
  //RedNegWP
  task::Async grabGoal(grabOnMove);
  legacy::forward(-475, 127, 4, 1100,-.3,0);
  pros::delay(200);
  legacy::turn(85, 127, 1, 1000, 0, 0);
  robot::intakeMtrs.move(127);
  task::Async checkIntake(checkIntakeMotor);
  legacy::forward(300, 85, 60, 1, 1500, 0, 0);
  legacy::forward(150, 65, 50, 1, 2500,0,0);
  pros::delay(500);
  legacy::forward(-380, 100, 127, 1, 1000,0,0);
  legacy::turn(50, 127,1, 2000,0,0);
  pros::delay(200);
  legacy::forward(350, 127, 1, 1000,0,0);
  legacy::turn(-80, 127, 1, 1000,0,0);
  pros::delay(100);
  legacy::forward(300, 127, 1, 1000,0,0);
  pros::delay(2000);
  legacy::turn(-120, 127,1, 1000,0,0);
  pros::delay(200);
  legacy::forward(400, 80, 1, 2500,0,0);
  _1028A::robot::intakeMtrs.move(-127);
  pros::delay(600);
  _1028A::robot::intakeMtrs.move(0);
  /*
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
  */
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
  legacy::forward(-380, -100, 127, 1, 1000,0,0);
  legacy::turn(-50, 127,1, 2000,0,0);
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
  legacy::forward(300, -85, 60, 1, 1500, 0, 0);
  legacy::forward(150, -65, 40, 1, 2500,0,0);
  pros::delay(500);
  legacy::forward(-490, -100, 127, 1, 3000,0,0);
  legacy::turn(-42, 127,1, 1000,0,0);
  pros::delay(200);
  legacy::forward(350, 127, 1, 1000,0,0);
  pros::delay(2000);
  legacy::turn(120, 127,1, 1000,0,0);
  pros::delay(200);
  legacy::forward(450, 80, 1, 2500,0,0);
  _1028A::robot::intakeMtrs.move(-127);
  pros::delay(600);
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
    legacy::forward(-420, 0, 127, 1, 1000,0,0);
    pros::delay(50);
    legacy::turn(-43, 127,1 , 1000,0,0);
    pros::delay(50);
    legacy::forward(-180, 127, 2.5, 500,0,0);
    pros::delay(100);
    robot::mogo.set_value(1);
    pros::delay(300);
    legacy::turn(20, 127, 1, 1000, 0, 0);
    pros::delay(100);
    robot::intakeMtrs.move(127);
    legacy::forward(200, 127, 1, 1000,0,0);
    pros::delay(500);
    legacy::turn(160, 127, 1, 1000, 0, 0);
    robot::mogo.set_value(0);
    pros::delay(100);
    legacy::turn(-94, 127, 1, 1000,0,0);
    task::Async grabGOAL(grabwhileback);
    pros::delay(100);
    legacy::forward(-300, 127, 1, 1000,0,0);
    pros::delay(100);
    legacy::turn(60, 127,1 ,1000,0,0);
   
  }
  else if(autonSelect == 6){
    legacy::forward(490, 0, 127, 3, 1000,0,0);
    robot::stick.set_value(1);
    pros::delay(800);
    legacy::forward(-300, 127, 1, 1000,0,0);
    robot::stick.set_value(0);
    pros::delay(400);
    legacy::turn(120, 127, 1, 1000,0,0);
    pros::delay(200);
    task::Async GRABGoal(grabonBack3);
    legacy::forward(-480, 120, 80, 1, 2000,0,0);
    robot::mogo.set_value(1);
    pros::delay(200);
    robot::intakeMtrs.move(127);
    legacy::turn(90, 127, 1, 1000,0,0);
    pros::delay(200);
    legacy::forward(200, 127, 1, 1000,0,0);
    pros::delay(900);
    robot::intakeMtrs.move(0);
    legacy::turn(-25, 127, 1, 1000,0,0);
    pros::delay(100);
    legacy::forward(-400, -50, 127, 1, 1000,0,0);
    /*
    pros::delay(400);
    robot::intakeMtrs.move(127);
    pros::delay(1000);
    robot::intakeMtrs.move(0);
    legacy::turn(-45, 127, 1, 1000,0,0);
    pros::delay(200);
    legacy::forward(-480, -50, 127, 1, 1000,0,0);
    */
    
  } 
  else if (autonSelect == 7){
    //RedSIGWP
    robot::intakeMtrs.move(-127);
    pros::delay(315);
    robot::intakeMtrs.move(0);
    task::Async fwd(armsDownonFWD);
    legacy::forward(250, 0, 127, 1.5, 700,0,0);
    pros::delay(200);
    legacy::forward(-170, 127, 3, 600,0,0);
    pros::delay(50);
    legacy::turn(65, 127, 1, 900,0,0);
    pros::delay(50);
    
    legacy::forward(-500, 71, 100, 1, 900,0,0);
    legacy::forward(-100, 71, 30, 1, 500,0,0);
    robot::mogo.set_value(1);
    robot::intakeMtrs.move(40);
    pros::delay(100);
    legacy::turn(52, 127, 1, 600,0,0);
    pros::delay(50);
    legacy::forward(280, 127, 1, 700,0,0);
    robot::stick.set_value(1);
    pros::delay(200);
    legacy::forward(-170, 127, 1, 700,0,0);
    robot::stick.set_value(0);
    pros::delay(200);
    legacy::turn(179, 127, 1, 900,0,0);
    pros::delay(50);
    robot::intakeMtrs.move(127);
    legacy::forward(280, 127, 1, 800,0,0);
    pros::delay(400);
    legacy::forward(-180, 127, 1, 800,0,0);
    pros::delay(50);
    legacy::turn(42, 127, 1, 1000,0,0);
    pros::delay(200);
    legacy::forward(200, 127, 1, 1000,0,0);
    pros::delay(50);
    legacy::turn(-100, 127, 1, 1000,0,0);
    pros::delay(100);
    legacy::forward(300, 70, 1, 1500, 0,0);
    robot::intakeMtrs.move(-127);
    pros::delay(1000);
    robot::intakeMtrs.move(0);
    /*
    pros::delay(50);
    task::Async dropFor(dropOnWay);
    legacy::forward(500, 0, 127, 1, 2000,0,0);
    pros::delay(50);
    robot::mogo.set_value(0);
    legacy::turn(60, 127, 1, 1000,0.4,0);

    pros::delay(50);
    legacy::forward(-250, 127, 1, 800,0,0);
    task::Async gOaLGrab(goalGrabonBack5);
    pros::delay(200);
    legacy::turn(180, 127, 2, 700,0,0);
    pros::delay(50);
    
    legacy::forward(200, 127, 5, 600,0,0);
    */

  }
  else if (autonSelect == 8){
    robot::intakeMtrs.move(-127);
    pros::delay(350);
    robot::intakeMtrs.move(0);
    task::Async fwd(armsDownonFWD);
    legacy::forward(250, 0, 127, 1.5, 700,0,0);
    pros::delay(200);
    legacy::forward(-170, 127, 3, 600,0,0);
    pros::delay(50);
    legacy::turn(60, 127, 1, 700,0,0);
    pros::delay(50);
    
    legacy::forward(-500, 71, 100, 1, 900,0,0);
    legacy::forward(-100, 71, 30, 1, 500,0,0);
    robot::mogo.set_value(1);
    robot::intakeMtrs.move(40);
    pros::delay(100);
    legacy::turn(52, 127, 1, 600,0,0);
    pros::delay(50);
    legacy::forward(280, 127, 1, 700,0,0);
    robot::stick.set_value(1);
    pros::delay(200);
    legacy::forward(-170, 127, 1, 700,0,0);
    robot::stick.set_value(0);
    robot::intakeMtrs.move(127);
    pros::delay(200);
    legacy::forward(170, 127, 1, 1000,0,0);
    pros::delay(200);
    legacy::forward(-170, 127, 1, 1000,0,0);
    legacy::turn(179, 127, 1, 900,0,0);
    pros::delay(50);
    robot::intakeMtrs.move(127);
    legacy::forward(280, 127, 1, 800,0,0);
    pros::delay(400);
    legacy::forward(-180, 127, 1, 800,0,0);
    pros::delay(50);
    legacy::turn(42, 127, 1, 1000,0,0);
    pros::delay(200);
    legacy::forward(200, 127, 1, 1000,0,0);

  }
  else if (autonSelect == 10){
    //RedSIGWP
    robot::intakeMtrs.move(-127);
    pros::delay(350);
    robot::intakeMtrs.move(0);
    task::Async fwd(armsDownonFWD);
    legacy::forward(250, 0, 127, 1.5, 700,0,0);
    pros::delay(200);
    legacy::forward(-170, 127, 3, 600,0,0);
    pros::delay(50);
    legacy::turn(71, 127, 1, 700,0,0);
    pros::delay(50);
    
    legacy::forward(-500, 71, 100, 1, 900,0,0);
    legacy::forward(-100, 71, 30, 1, 500,0,0);
    robot::mogo.set_value(1);
    robot::intakeMtrs.move(40);
    pros::delay(100);
    legacy::turn(165, 127, 1, 900,0,0);
    pros::delay(50);
    robot::intakeMtrs.move(127);
    legacy::forward(280, 127, 1, 800,0,0);
    pros::delay(100);
    legacy::forward(-170, 127, 1, 800,0,0);
    legacy::turn(65.5, 127, 1, 600,0,0);
    pros::delay(50);
    legacy::forward(320, 127, 1, 700,0,0);

    //robot::stick.set_value(1);
    //pros::delay(100);
    //task::Async turnStick(stickuponturn);
    //legacy::turn(0, 127, 1, 1000,0,0);
    


  }
  else if (autonSelect == 11){
    task::Async checkke(checkIntakeMotor);
    robot::intakeMtrs.move(-127);
    pros::delay(320);
    robot::intakeMtrs.move(0);
    legacy::forward(640, 80, 1.5,2500,0,.5);
    robot::intakeMtrs.move(127);
    pros::delay(1000);
    legacy::turn(65, 127, 1, 1000,0,0);
    pros::delay(200);
    task::Async GrAbOnBack(goalGrabonBack9);
    legacy::forward(-500, 65, 100, 1, 1000,0,0);
    pros::delay(100);
    legacy::turn(5, 127, 1, 1000,0,0);
    pros::delay(100);
    legacy::forward(300, 110, 1, 1000,0,0);
    pros::delay(100);
    legacy::turn(60, 127, 1, 1000,0,0);
    pros::delay(200);
    robot::intakeMtrs.move(0);
    legacy::forward(300, 127, 1, 1000,0,0);
    
  }
  else if (autonSelect == 12){
    //Skills
    task::Async checkIntake(checkIntakeMotor);
    robot::intakeMtrs.move(-127);
    pros::delay(320);
    robot::intakeMtrs.move(0);
    task::Async fwd(armsDownonFWD);
    legacy::forward(250, 127, 1.5, 700,0,0);
    pros::delay(200);
    legacy::forward(-120, 127, 3, 600,0,0);
    pros::delay(50);
    legacy::turn(0, 127, 1, 1000,0,0);
    pros::delay(100);
    legacy::forward(180, 127, 1, 800,0,0);
    pros::delay(200);
    legacy::turn(90, 127, 1, 1000,0,0);
    pros::delay(200);
    task::Async grabwhileBack(grabwhileback);
    legacy::forward(-300, 80, 1, 1000,0,0);
    pros::delay(50);
    legacy::forward(80, 127, 1, 300,0,0);
    pros::delay(200);
    legacy::turn(0, 127, 1, 1000,0,0);
    robot::intakeMtrs.move(127);
    pros::delay(200);
    robot::intakeMtrs.move(127);
    legacy::forward(300, 127, 1, 1000,0,0);
    pros::delay(700);
    robot::intakeMtrs.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
    robot::intakeMtrs.move(0);
    /*
    legacy::turn(50, 127, 1, 1000,0,0);
    task::Async StartIntake(startIntake);
    pros::delay(200);
    legacy::forward(500, 50, 127, 1, 1000,0,0);
    robot::intakeMtrs.move(127);
    pros::delay(1200);
    robot::intakeMtrs.move(0);
    legacy::forward(-540, 50, 127, 1, 1000,0,0);
    robot::intakeMtrs.move(127);
    pros::delay(200);
    */
    legacy::turn(-38, 127, 1, 1000,0,0);
    pros::delay(200);
    legacy::forward(350, 127, 1, 1000,0,0);
    checkIntake.stopTask();
    _1028A::task::Async diskIndex (Macro);  
    pros::delay(200);
    _1028A::legacy::turn(-91, 127, 1, 1000,0,0);
    pros::delay(200);
    _1028A::legacy::forward(135, 127, 1, 1000,0,0);
    int starttime = pros::millis();
    while (1){
      if(diskIndex.isComplete()){
        break;
        
      }
      else if (pros::millis() - starttime > 3000){
        diskIndex.forceStop();
        break;
      }
      pros::delay(5);
    }
   _1028A::task::Async LiftArms(liftArms);
   starttime = pros::millis();
    while (1){
      if(LiftArms.isComplete()){
        break;
        
      }
      else if (pros::millis() - starttime > 2000){
        LiftArms.forceStop();
        break;
      }
      pros::delay(5);
    }
    legacy::forward(180, 80, 1, 400,0,0);
    pros::delay(200);
    robot::intakeMtrs.move(127);
    pros::delay(200);
    robot::leftfront.tare_position();
    legacy::forward(-190, 127, 1, 1000,0,0);
    checkIntake.startTask();
    pros::delay(200);
    legacy::turn(-181, 127, 1, 1000,0,0);
    pros::delay(200);
    legacy::forward(400,  127, 3, 1000,0,0);
    legacy::forward(500,  20, 1, 3300,0,0);
    pros::delay(200);
    legacy::forward(-190, 90, 1, 700,0,0);
    pros::delay(200);
    legacy::turn(-90, 127, 1, 800,0,0);
    pros::delay(200);
    legacy::forward(150, 60, 1, 800,0,0);
    pros::delay(1000);
    legacy::forward(-170, 127, 1, 800,0,0);
    pros::delay(800);
    robot::intakeMtrs.move(0);
    legacy::turn(45, 127, 1, 1000,0,0);
    pros::delay(200);
    legacy::forward(-300, 45, 127, 1, 1000,0,0);
    robot::intakeMtrs.move(-50);
    robot::mogo.set_value(0);
    legacy::turn(45, 127, 1, 1000,0.8,0);
    pros::delay(200);
    legacy::forward(240, 127, 1, 1000,0,0);
    robot::intakeMtrs.move(0);
    pros::delay(200);
    legacy::turn(-89, 127, 1, 1000,0,0);
    pros::delay(200);
    task::Async grabonback(grabonBack2);
    legacy::forward(-680, -90, 100, 5, 2000,0,.9);
    legacy::forward(-280, -90, 40, 1, 2000,0,.9);
    legacy::forward(90, 127, 1, 500,0,0);
    robot::mogo.set_value(1);
    pros::delay(200);
    legacy::turn(0, 127, 1, 1000,0,0);
    pros::delay(200);
    robot::intakeMtrs.move(127);
    legacy::forward(280, 127, 1, 1000,0,0);
    pros::delay(200);
    legacy::turn(39, 127, 1, 1000,0,0);
    pros::delay(200);
    legacy::forward(390, 127, 1, 1500,0,0);

    pros::delay(200);
    legacy::turn(90, 127, 1, 1000,0,0);
    _1028A::task::Async diskINdex (Macro);  
    pros::delay(200);
    _1028A::legacy::forward(150, 127, 1, 1000,0,0);
    starttime = pros::millis();
    checkIntake.stopTask();
    while (1){
      if(diskINdex.isComplete()){
        break;
        
      }
      else if (pros::millis() - starttime > 2000){
        diskINdex.forceStop();
        break;
      }
      pros::delay(5);
    }
   _1028A::task::Async LiftARms(liftArms);
   starttime = pros::millis();
    while (1){
      if(LiftARms.isComplete()){
        break;
        
      }
      else if (pros::millis() - starttime > 2000){
        LiftARms.forceStop();
        break;
      }
      pros::delay(5);
    }
    legacy::forward(180, 80, 1, 500,0,0);
    pros::delay(200);
    robot::intakeMtrs.move(127);
    pros::delay(400);
    legacy::forward(-190, 80, 1, 900,0,0);
    pros::delay(200);
    checkIntake.startTask();
    legacy::turn(180, 127, 1, 1000,0,0);
    pros::delay(200);
    robot::intakeMtrs.move(127);
    pros::delay(400);
    legacy::forward(400,  127, 3, 1000,0,0);
    legacy::forward(500,  20, 1, 3000,0,0);
    pros::delay(200);
    legacy::forward(-190, 90, 1, 800,0,0);
    pros::delay(200);
    legacy::turn(90, 127, 1, 1000,0,0);
    pros::delay(200);
    legacy::forward(150, 60, 1, 1000,0,0);
    pros::delay(1000);
    legacy::forward(-180, 127, 1, 700,0,0);
    pros::delay(800);
    robot::intakeMtrs.move(0);
    legacy::turn(-45, 127, 1, 1000,0,0);
    pros::delay(200);
    legacy::forward(-300, -45, 127, 1, 1000,0,0);
    robot::intakeMtrs.move(-50);
    robot::mogo.set_value(0);
    pros::delay(100);
    legacy::turn(-10, 127, 2, 800,0,0);    
    legacy::forward(600, -10, 127, 2, 1000,0,0);
    legacy::forward(1000, -40, 127, 1, 2500,0,0);
    legacy::turn(-120, 127, 1, 1000,0,0);
    pros::delay(200);
    legacy::forward(-980, robot::inertialReg.get_rotation(), 127, 4, 1200,0,0);
    robot::intakeMtrs.move(127);
    legacy::forward(1800, -90, 127, 4, 5000,0,0);
    //legacy::forward(900, -120, 127, 4, 2000,0,0);
     //legacy::forward(900, 120, 127, 4, 2000,0,0);
    /*
    pros::delay(200);
    legacy::forward(600, 127, 1, 1000,0,0);
    legacy::turn(-70, 127, 1, 1000,0,0);
    pros::delay(200);
    legacy::forward(600, -70, 127, 1, 1000,0,0);
    */
    

  }
}