#include "1028A/sensors.h"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "main.h"
#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"
#define leftfrontpt 9
#define leftmidpt 7
#define leftbackpt 12
#define rightfrontpt 10
#define rightmidpt 5
#define rightbackpt 16
#define inertialpt 21
#define inakept 6
#define kickerpt 17
#define leftencpt 1
#define rightencpt 2

namespace _1028A::robot {
extern pros::Motor leftfront;
extern pros::Motor leftmid;
extern pros::Motor leftback;
extern pros::Motor_Group leftMtrs;
extern pros::Motor rightfront;
extern pros::Motor rightmid;
extern pros::Motor rightback;
extern pros::Motor_Group rightMtrs;
extern pros::Motor kicker;
extern pros::Motor intake;
extern pros::IMU inertial;
extern pros::Rotation leftEnc;
extern pros::Rotation rightEnc;
extern pros::Controller master;
extern pros::ADIDigitalOut flapL;
extern pros::ADIDigitalOut flapR;
extern pros::ADIDigitalOut climb_set1;
extern pros::ADIDigitalOut climb_set2;
extern pros::ADIDigitalOut stick;

extern lemlib::Drivetrain_t drivetrain;
extern lemlib::TrackingWheel leftTracker;
extern lemlib::TrackingWheel rightTracker;
extern lemlib::OdomSensors_t odomSensors;
extern lemlib::ChassisController_t lateralController;
extern lemlib::ChassisController_t angularController;
extern lemlib::Chassis chassis;

} // namespace _1028A::robot