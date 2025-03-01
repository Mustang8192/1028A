#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "main.h"
#include "pros/adi.hpp"
#include "lemlib/api.hpp"
#include "pros/distance.hpp"
#include "pros/optical.hpp"
#include "1028A/mcl.h"

#define leftFrontpt 18
#define leftMidpt 19
#define leftBackpt 20
#define rightFrontpt 8
#define rightMidpt 9
#define rightBackpt 10
#define intakept 17
#define LBLPort 12
#define LBRPort 11
#define LBSPort 13
#define Verticalpt 7
#define Horizontalpt 21
#define inertialpt 16
#define mogoPort 'g'
#define distancePort 14
#define opticalPort 4

namespace _1028A::robot{
    extern pros::Motor leftFront;
    extern pros::Motor leftMid;
    extern pros::Motor leftBack;
    extern pros::MotorGroup leftMtrs;
    extern pros::Motor rightFront;
    extern pros::Motor rightMid;
    extern pros::Motor rightBack;
    extern pros::MotorGroup rightMtrs;
    extern pros::Motor intake;
    extern pros::Motor LBL;
    extern pros::Motor LBR;
    extern pros::MotorGroup LB;
    extern pros::Rotation LBS;
    extern pros::adi::DigitalIn LBSLimit;
    extern pros::adi::DigitalIn CaliSwitch;
    extern pros::Rotation Vertical;
    extern pros::Rotation Horizontal;
    extern pros::Imu inertial;
    extern pros::Distance distance;
    extern pros::Optical optical;
    extern pros::adi::DigitalOut mogo;
    extern pros::adi::DigitalOut stick;
    extern pros::Controller master;

    extern lemlib::Drivetrain drivetrain;
    extern lemlib::TrackingWheel vertical;
    extern lemlib::TrackingWheel horizontal;
    extern lemlib::OdomSensors sensors;
    extern lemlib::ControllerSettings lateralController;
    extern lemlib::ControllerSettings angularController;
    extern lemlib::Chassis chassis;
    extern lemlib::ExpoDriveCurve driveCurve;
    extern lemlib::ExpoDriveCurve turnCurve;
}