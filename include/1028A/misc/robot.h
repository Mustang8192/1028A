#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "main.h"
#include "pros/adi.hpp"
#include "pros/distance.hpp"

#define leftfrontpt 1
#define leftmidpt 2
#define leftbackpt 3
#define rightfrontpt 10
#define rightmidpt 9
#define rightbackpt 8
#define intakeLpt 6
#define intakeRpt 7
#define inertialOdompt 21
#define inertialRegpt 4
#define horizontalencodpt 15
#define ringpt 17
#define ringLpt 16
#define opticalpt 11

namespace _1028A::robot {
extern pros::Motor leftfront;
extern pros::Motor leftmid;
extern pros::Motor leftback;
extern pros::Motor_Group leftMtrs;
extern pros::Motor rightfront;
extern pros::Motor rightmid;
extern pros::Motor rightback;
extern pros::Motor_Group rightMtrs;
extern pros::Motor intakeL;
extern pros::Motor intakeR;
extern pros::Motor_Group intakeMtrs;
extern pros::ADIDigitalOut mogo;
extern pros::ADIDigitalOut stick;
extern pros::IMU inertialOdom;
extern pros::IMU inertialReg;
extern pros::Rotation horizontalencod;
extern pros::Distance ring;
extern pros::Distance ringL;
extern pros::Optical optical;
extern pros::Controller master;
extern pros::ADIDigitalIn limitSwitch;

extern lemlib::Drivetrain drivetrain;
extern lemlib::TrackingWheel horizontalEncoder;
extern lemlib::OdomSensors sensors;
extern lemlib::ControllerSettings lateral_controller;
extern lemlib::ControllerSettings angular_controller;
extern lemlib::Chassis chassis;
} // namespace _1028A::robot