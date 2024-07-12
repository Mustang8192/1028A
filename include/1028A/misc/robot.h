
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "main.h"
#include "pros/adi.hpp"

#define leftfrontpt 1
#define leftmidpt 2
#define leftbackpt 3
#define rightfrontpt 10
#define rightmidpt 9
#define rightbackpt 8
#define intakept 5
#define conveyorpt 7
#define inertialpt 20
#define horizontalencodpt 19

namespace _1028A::robot {
extern pros::Motor leftfront;
extern pros::Motor leftmid;
extern pros::Motor leftback;
extern pros::Motor_Group leftMtrs;
extern pros::Motor rightfront;
extern pros::Motor rightmid;
extern pros::Motor rightback;
extern pros::Motor_Group rightMtrs;
extern pros::Motor intake;
extern pros::Motor conveyor;
extern pros::Motor_Group intakeMtrs;
extern pros::ADIDigitalOut mogo;
extern pros::ADIDigitalOut Ilift;
extern pros::ADIDigitalOut HGlift;
extern pros::IMU inertial;
extern pros::Rotation horizontalencod;
extern pros::Controller master;

extern lemlib::Drivetrain drivetrain;
extern lemlib::TrackingWheel horizontalEncoder;
extern lemlib::OdomSensors sensors;
extern lemlib::ControllerSettings lateral_controller;
extern lemlib::ControllerSettings angular_controller;
extern lemlib::Chassis chassis;
} // namespace _1028A::robot