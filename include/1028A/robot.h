#include "1028A/chassis/chassis.hpp"
#include "1028A/chassis/trackingWheel.hpp"
#include "1028A/sensors.h"
#include "main.h"
#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"
#define leftfrontpt 7
#define leftmidpt 11
#define leftbackpt 12
#define rightfrontpt 10
#define rightmidpt 6
#define rightbackpt 19
#define inertialpt 14
#define inakept 5
#define kickerpt 17
#define leftencpt 9
#define rightencpt 7
#define adiExtPt 8
#define flapLpt 'B'
#define flapRpt 'A'
#define climb_set1pt 'E'
#define climb_set2pt 'G'
#define backLpt 'D'
#define backRpt 'C'

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
extern pros::ADIDigitalOut backL;
extern pros::ADIDigitalOut backR;

extern _1028A::Drivetrain_t drivetrain;
extern _1028A::TrackingWheel leftTracker;
extern _1028A::TrackingWheel rightTracker;
extern _1028A::OdomSensors_t odomSensors;
extern _1028A::ChassisController_t lateralController;
extern _1028A::ChassisController_t angularController;
extern _1028A::Chassis chassis;

} // namespace _1028A::robot