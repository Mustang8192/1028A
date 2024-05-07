#include "1028A/misc/robot.h"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include <cstddef>

pros::Motor _1028A::robot::leftfront(leftfrontpt, pros::E_MOTOR_GEARSET_06,
                                     true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor _1028A::robot::leftmid(leftmidpt, pros::E_MOTOR_GEARSET_06, true,
                                   pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor _1028A::robot::leftback(leftbackpt, pros::E_MOTOR_GEARSET_06, true,
                                    pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor_Group _1028A::robot::leftMtrs({leftfrontpt, leftmidpt, leftbackpt});

pros::Motor _1028A::robot::rightfront(rightfrontpt, pros::E_MOTOR_GEARSET_06,
                                      false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor _1028A::robot::rightmid(rightmidpt, pros::E_MOTOR_GEARSET_06, false,
                                    pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor _1028A::robot::rightback(rightbackpt, pros::E_MOTOR_GEARSET_06,
                                     false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor_Group _1028A::robot::rightMtrs({rightfrontpt, rightmidpt,
                                            rightbackpt});
pros::IMU _1028A::robot::inertial(inertialpt);
pros::Rotation _1028A::robot::hortzencod(hortzencodpt);

pros::Controller _1028A::robot::master(pros::E_CONTROLLER_MASTER);

lemlib::Drivetrain _1028A::robot::drivetrain(&_1028A::robot::leftMtrs,
                                             &_1028A::robot::rightMtrs, 10,
                                             lemlib::Omniwheel::NEW_325, 450,
                                             2);

lemlib::TrackingWheel
    _1028A::robot::horizontalTracker(&_1028A::robot::hortzencod,
                                     lemlib::Omniwheel::NEW_2, -5);

lemlib::OdomSensors _1028A::robot::sensors(nullptr, nullptr, &horizontalTracker,
                                           nullptr, &inertial);

lemlib::ControllerSettings _1028A::robot::lateral_controller(
    10,  // proportional gain (kP)
    0,   // integral gain (kI)
    3,   // derivative gain (kD)
    3,   // anti windup
    1,   // small error range, in inches
    100, // small error range timeout, in milliseconds
    3,   // large error range, in inches
    500, // large error range timeout, in milliseconds
    20   // maximum acceleration (slew)
);

lemlib::ControllerSettings _1028A::robot::angular_controller(
    2,   // proportional gain (kP)
    0,   // integral gain (kI)
    10,  // derivative gain (kD)
    3,   // anti windup
    1,   // small error range, in degrees
    100, // small error range timeout, in milliseconds
    3,   // large error range, in degrees
    500, // large error range timeout, in milliseconds
    0    // maximum acceleration (slew)
);

lemlib::Chassis
    _1028A::robot::chassis(drivetrain,         // drivetrain settings
                           lateral_controller, // lateral PID settings
                           angular_controller, // angular PID settings
                           sensors             // odometry sensors
    );