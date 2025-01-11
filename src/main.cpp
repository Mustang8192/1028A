#include "main.h"
#include "1028A/api.h"


void initialize() {
	_1028A::misc::init();
	_1028A::robot::chassis.calibrate();
}


void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
	pros::Task DriveCTRL(_1028A::driver::driveCTRL);
	pros::Task IntakeCTRL(_1028A::driver::intakeCTRL);
	pros::Task MogoCTRL(_1028A::driver::mogoCTRL);
	pros::Task Lbmacro(_1028A::driver::lbmacro);
	pros::Task OdomRead(_1028A::driver::odomRead);
		
	while (true) {
		pros::delay(20);
	}
}