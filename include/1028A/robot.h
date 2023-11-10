#include "1028A/sensors.h"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "main.h"
#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#define leftfrontpt 9
#define leftmidpt 7
#define leftbackpt 17
#define rightfrontpt 8
#define rightmidpt 6
#define rightbackpt 20
#define verticalEncpt 11
#define horizontalEncpt 12
#define inertialpt 13
#define inakept 10
#define flywheelpt 16

namespace _1028A::robot {
extern pros::Motor leftfront;
extern pros::Motor leftmid;
extern pros::Motor leftback;
extern pros::Motor_Group leftMtrs;
extern pros::Motor rightfront;
extern pros::Motor rightmid;
extern pros::Motor rightback;
extern pros::Motor_Group rightMtrs;
extern pros::Motor flywheel;
extern pros::Motor intake;
extern pros::Rotation verticalEnc;
extern pros::Rotation horizontalEnc;
extern pros::IMU inertial;
extern pros::Controller master;
extern pros::ADIDigitalOut flapL;
extern pros::ADIDigitalOut flapR;
extern pros::ADIDigitalOut climb;

extern lemlib::Drivetrain_t drivetrain;
extern lemlib::TrackingWheel verticalTracker;
extern lemlib::TrackingWheel horizontalTracker;
extern lemlib::OdomSensors_t odomSensors;
extern lemlib::ChassisController_t lateralController;
extern lemlib::ChassisController_t angularController;
extern lemlib::Chassis chassis;

} // namespace _1028A::robot