#include "main.h"
#include "1028A/api.h"
#include "pros/rtos.hpp"

void initialize() {
  _1028A::utils::init();
  _1028A::robot::chassis.calibrate();
}

void disabled() { }

void competition_initialize() {}
ASSET(Disk6Seq1_txt);
ASSET(goal1ring_txt);
ASSET (test5_txt);

void autonomous() {
  
  /*_1028A::comp::auton();*/
}

void CheckIntakeMotor() {
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

void MAcro(){
    _1028A::robot::intakeMtrs.move(127);
      while (1){
        if (_1028A::robot::ringL.get() < 40){
          pros::delay(30);
          _1028A::robot::intakeMtrs.move(70);
          while (1){
            if (_1028A::robot::ring.get() < 40){
              pros::delay(10);
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

void Load(){
  pros::delay(1000);
  while (1){
        if (_1028A::robot::ringL.get() < 40){
          pros::delay(30);
          _1028A::robot::intakeMtrs.move(60);
          while (1){
            if (_1028A::robot::ring.get() < 40){
              //pros::delay(10);
              _1028A::robot::intakeMtrs.move(0);
              pros::delay(200);
            _1028A::robot::intakeMtrs.move(-127);
            pros::delay(1200);
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


void opcontrol() {
  _1028A::task::Async checkIntake(CheckIntakeMotor);
  _1028A::task::Async Macros(_1028A::comp::driver::macros);

  _1028A::robot::intakeMtrs.move(127);
  pros::delay(210);
  _1028A::robot::intakeMtrs.move(0);
  _1028A::robot::chassis.moveToPoint(0, 12, 1000);
  _1028A::robot::chassis.turnToHeading(90, 1000);
  _1028A::robot::chassis.moveToPoint(-28, 14, 1000, {.forwards = false}, false);
  _1028A::robot::mogo.set_value(1);
  pros::delay(200);
  _1028A::robot::chassis.moveToPose(-27.333029, 35.108574, -35.314808, 3000, {.minSpeed=40}, false);
  _1028A::robot::chassis.moveToPose(-46.069965, 63.184368, -11.276176, 3000, {.minSpeed=40}, false);
  _1028A::robot::chassis.moveToPose(-52.281696, 86.281021, -16.772146, 3000, {.minSpeed=40}, false);
  checkIntake.forceStop();
  _1028A::robot::chassis.moveToPose(-46.749065, 67.600578, -15.664765, 3000, {.forwards = false}, false);
  _1028A::robot::chassis.turnToHeading(-90.5, 1000, {}, false);
  _1028A::task::Async LoadRing(Load);
  std::string str = "LeftFront Position: " + std::to_string(_1028A::robot::leftfront.get_position());
  _1028A::logger::info(str.c_str());
  _1028A::legacy::forwardnoKill(600, 127, 1, 1000, 0, 0);
  LoadRing.waitUntilComplete();
  pros::delay(100);
  _1028A::legacy::forwardnoKill(730, 127, 1, 1000, 0, 0);
  _1028A::robot::intakeMtrs.move(127);
  pros::delay(500);
  _1028A::legacy::forwardnoKill(550, 127, 1, 1000,0,0);
  _1028A::robot::chassis.turnToHeading(-180, 1000, {} , false);
  

  while (true) {
    pros::delay(200);
  }
}
