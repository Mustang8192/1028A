#include "api.h"
#pragma once
#define leftFrontPort 1
#define leftMidPort 2
#define leftBackPort 3
#define rightFrontPort 4
#define rightMidPort 5
#define rightBackPort 6
#define AuxLPort 7
#define AuxRPort 8
#define inertialPort 9
#define gpsPort 10
#define leftEncoderPort 11
#define rightEncoderPort 12
#define backEncoderPort 13

extern pros::Motor leftFront;
extern pros::Motor leftMid;
extern pros::Motor leftBack;
extern pros::Motor_Group leftDrive;
extern pros::Motor rightFront;
extern pros::Motor rightMid;
extern pros::Motor rightBack;
extern pros::Motor_Group rightDrive;
extern pros::Motor AuxL;
extern pros::Motor AuxR;
extern pros::Motor_Group Aux;
extern pros::IMU inertial;
extern pros::GPS gps;
extern pros::Rotation leftEncoder;
extern pros::Rotation rightEncoder;
extern pros::Rotation backEncoder;
extern pros::Controller mainController;

extern _1028A::TrackingWheel leftTracker;
extern _1028A::TrackingWheel rightTracker;
extern _1028A::TrackingWheel backTracker;
extern _1028A::Drivetrain_t drivetrain;
extern _1028A::OdomSensors_t OdomSensors;
extern _1028A::ChassisController_t lateralController;
extern _1028A::ChassisController_t angularController;
extern _1028A::Chassis chassis;