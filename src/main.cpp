#include "main.h"
#include "1028A/api.h"
#include "pros/rtos.hpp"

void initialize() {
  _1028A::utils::init();
  _1028A::robot::chassis.calibrate();
}

void disabled() { }

void competition_initialize() {}
ASSET(pose1_txt);

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
              pros::delay(10);
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

void intakeKill(){
  _1028A::robot::intakeMtrs.move(127);
  while (1){
        if (_1028A::robot::ringL.get() < 40){
          _1028A::robot::intakeMtrs.move(0);
          
          break;
        }
          pros::delay(5);
  }

}


void opcontrol() {
  _1028A::task::Async Macros(_1028A::comp::driver::macros);
  _1028A::task::Async IntakeKill(intakeKill);
  _1028A::robot::chassis.moveToPose(-24.476562, 36.903278, -49.753448, 3000, {}, false);
  _1028A::robot::chassis.moveToPose(-22.048130, 12.335558, 31.558695, 3000, {.forwards=false}, false);
  _1028A::robot::mogo.set_value(1);
  pros::delay(200);
  _1028A::robot::intakeMtrs.move(127);
  _1028A::task::Async checkIntake(CheckIntakeMotor);
  _1028A::robot::chassis.moveToPose(-48.464626, 35.953781, -72.369186, 3000, {}, false);
  _1028A::robot::chassis.turnToHeading(-4.747673, 1000);
  _1028A::robot::chassis.moveToPose(-49.740234, 81.555046, -7.392817, 3000, {}, false);
  pros::delay(200);
  _1028A::robot::chassis.moveToPoint(-43.160156, 61.063358, 3000, {.forwards=false}, false);
  _1028A::robot::chassis.turnToHeading(-90, 1000);
  checkIntake.stopTask();
  _1028A::task::Async LoadMacro(Load);


  while (true) {
    pros::delay(200);
  }
}
