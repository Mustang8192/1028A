#include "1028A/sensors.h"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "main.h"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#define leftfrontpt 1
#define leftmidpt 2
#define leftbackpt 3
#define rightfrontpt 4
#define rightmidpt 5
#define rightbackpt 6
#define auxL11pt 7
#define auxL55pt 8
#define auxR11pt 9
#define auxR55pt 10
#define verticalEncpt 11
#define horizontalEncpt 12
#define inertialpt 13
#define inertialOdompt 14
#define gps1pt 15
#define gps2pt 16
#define opticalpt 17
#define inakept 18

namespace _1028A::robot {
extern pros::Motor leftfront;
extern pros::Motor leftmid;
extern pros::Motor leftback;
extern pros::Motor_Group leftMtrs;
extern pros::Motor_Group leftMtrswPTO;
extern pros::Motor rightfront;
extern pros::Motor rightmid;
extern pros::Motor rightback;
extern pros::Motor_Group rightMtrs;
extern pros::Motor_Group rightMtrswPTO;
extern pros::Motor auxL11;
extern pros::Motor auxL55;
extern pros::Motor auxR11;
extern pros::Motor auxR55;
extern pros::Motor intake;
extern pros::Rotation verticalEnc;
extern pros::Rotation horizontalEnc;
extern pros::IMU inertial;
extern pros::IMU inertialOdom;
extern pros::GPS gps1;
extern pros::GPS gps2;
extern GPSRedundantSensor gps;
extern pros::Optical optical;
extern pros::Controller master;

extern lemlib::Drivetrain_t drivetrain;
extern lemlib::TrackingWheel verticalTracker;
extern lemlib::TrackingWheel horizontalTracker;
extern lemlib::OdomSensors_t odomSensors;
extern lemlib::ChassisController_t lateralController;
extern lemlib::ChassisController_t angularController;
extern lemlib::Chassis chassis;

} // namespace _1028A::robot