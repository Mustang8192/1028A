#include "main.h"
#include "1028A/api.h"


void initialize() {
	_1028A::misc::init();
	_1028A::robot::chassis.calibrate();
}


void disabled() {
	_1028A::logger::info("Disabled");
}

void competition_initialize() {
	_1028A::logger::info("Competition Initialize");
	//_1028A::misc::waitForCalibrate();
}

void autonomous() {
	_1028A::logger::info("Auton");
	_1028A::auton::auton();
}

void opcontrol() {
	_1028A::auton::autonStop = 1;
	_1028A::logger::info("Opcontrol");

	pros::Task DriveCTRL(_1028A::driver::driveCTRL);
	pros::Task IntakeCTRL(_1028A::driver::intakeCTRL);
	pros::Task MogoCTRL(_1028A::driver::mogoCTRL);
	pros::Task Lbmacro(_1028A::driver::lbmacro);
	pros::Task StickCTRL(_1028A::driver::stickCTRL);
	pros::Task OdomRead(_1028A::driver::odomRead);
	
	while (true) {
		// _1028A::Particle bestParticle = _1028A::robot::localizer.updateLocalization();
		// std::string l = "X: " + std::to_string(bestParticle.x) + " Y: " + std::to_string(bestParticle.y) + " Theta: " + std::to_string(bestParticle.theta) + " Weight: " + std::to_string(bestParticle.weight);
		// _1028A::logger::info(l.c_str());
		// //print odom as well
		// std::string l2 = "X: " + std::to_string(_1028A::robot::chassis.getPose().x) + " Y: " + std::to_string(_1028A::robot::chassis.getPose().y) + " Theta: " + std::to_string(_1028A::robot::chassis.getPose().theta);
		// _1028A::logger::info(l2.c_str());
		//std::string l3 = "Distance: " + std::to_string(disk.get());
		//_1028A::logger::info(l3.c_str());
		pros::delay(1000);
	}
}