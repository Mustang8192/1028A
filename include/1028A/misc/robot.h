#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "main.h"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/motors.hpp"

#define leftFrontPort 18
#define leftMidPort 19
#define leftBackPort 20
#define rightFrontPort 8
#define rightMidPort 9
#define rightBackPort 10
#define intakePort 17
#define LBLPort 12
#define LBRPort 11
#define LBSPort 13
#define VerticalPort 7
#define horizontalPort 21
#define inertialPort 16
#define inertialOdomPort 1
#define mogoPort 'g'

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
extern pros::ADIDigitalOut mogo;
extern pros::ADIDigitalIn LBSwitch;
extern pros::Rotation LBS;
extern pros::Motor LBL;
extern pros::Motor LBR;
extern pros::Motor_Group LB;
extern pros::IMU inertial;
extern pros::Rotation verticalencod;
extern pros::Rotation horizontalencod;
extern pros::Controller master;

extern lemlib::Drivetrain drivetrain;
extern lemlib::TrackingWheel vertTracking;
extern lemlib::TrackingWheel horizontalTracking;
extern lemlib::OdomSensors sensors;
extern lemlib::ControllerSettings lateralController;
extern lemlib::ControllerSettings angularController;
extern lemlib::Chassis chassis;
extern lemlib::ExpoDriveCurve driveCurve;
extern lemlib::ExpoDriveCurve turnCurve;
} // namespace _1028A::robot