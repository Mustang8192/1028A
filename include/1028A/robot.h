#include "1028A/chassis/chassis.hpp"
#include "1028A/chassis/trackingWheel.hpp"
#include "1028A/sensors.h"
#include "main.h"
#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"
#define leftfrontpt 5
#define leftmidpt 2
#define leftbackpt 11
#define rightfrontpt 4
#define rightmidpt 3
#define rightbackpt 17
#define inertialpt 21
#define inakept 1
#define kickerpt 16
#define leftencpt 9
#define rightencpt 7

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

extern _1028A::Drivetrain_t drivetrain;
extern _1028A::TrackingWheel leftTracker;
extern _1028A::TrackingWheel rightTracker;
extern _1028A::OdomSensors_t odomSensors;
extern _1028A::ChassisController_t lateralController;
extern _1028A::ChassisController_t angularController;
extern _1028A::Chassis chassis;

} // namespace _1028A::robot