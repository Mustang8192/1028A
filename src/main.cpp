#include "main.h"
#include "1028A/api.h"
#include "pros/rtos.hpp"

void initialize() {
  _1028A::utils::init();
  _1028A::robot::chassis.calibrate();
}

void disabled() { }

void competition_initialize() {}

ASSET(goal1ring_txt);

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

void opcontrol() {
  /*
  _1028A::task::Async DriveCTRL(_1028A::comp::driver::driverCTRL);
  _1028A::task::Async IntakeCTRL(_1028A::comp::driver::intakeCTRL);
  _1028A::task::Async MogoCTRL(_1028A::comp::driver::mogoCTRL);
  _1028A::task::Async HGCTLR(_1028A::comp::driver::HGCTRL);
  _1028A::task::Async Stick (_1028A::comp::driver::stickCTRL);
  _1028A::task::Async Assistance(_1028A::comp::driver::assistance);
  */
  _1028A::task::Async checkIntake(CheckIntakeMotor);
  _1028A::task::Async Macros(_1028A::comp::driver::macros);

  _1028A::robot::intakeMtrs.move(127);
  pros::delay(220);
  _1028A::robot::intakeMtrs.move(0);
  _1028A::robot::chassis.moveToPoint(0, 12, 1000);
  _1028A::robot::chassis.turnToHeading(90, 1000);
  _1028A::robot::chassis.moveToPoint(-28, 12, 1000, {.forwards = false}, false);
  _1028A::robot::chassis.setPose({-47.283,26.366,180});
  _1028A::robot::mogo.set_value(1);
  pros::delay(200);
  _1028A::robot::chassis.follow(goal1ring_txt, 10, 10000, true, false);
  pros::delay(200);
  _1028A::robot::chassis.moveToPoint(9, 40.730309, 2000, {.forwards = false}, false);
  _1028A::robot::chassis.turnToHeading(0, 1000, {}, false);
  _1028A::task::Async diskIndex (MAcro);  
  _1028A::legacy::forward(180, 127, 1, 1000,0,0);
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
    _1028A::robot::intakeMtrs.move(-127);
    pros::delay(800);
    _1028A::robot::intakeMtrs.move(0);
    _1028A::legacy::forward(180, 80, 1, 400,0,0);
    _1028A::robot::intakeMtrs.move(127);

  while (true) {
    pros::delay(200);
  }
}
