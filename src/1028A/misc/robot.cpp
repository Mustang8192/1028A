#include "1028A/misc/robot.h"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
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
pros::Motor _1028A::robot::intakeL(intakeLpt, pros::E_MOTOR_GEARSET_06, true,
                                  pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor _1028A::robot::intakeR(intakeRpt, pros::E_MOTOR_GEARSET_06, false,
                                    pros::E_MOTOR_ENCODER_DEGREES);
pros::ADIDigitalOut _1028A::robot::mogo('h');
pros::ADIDigitalOut _1028A::robot::stick('c');
pros::ADIAnalogIn _1028A::robot::LineL ('e');
pros::ADIAnalogIn _1028A::robot::LineR('f');
pros::Motor_Group _1028A::robot::intakeMtrs({intakeLpt, intakeRpt});
pros::IMU _1028A::robot::inertialOdom(inertialOdompt);
pros::IMU _1028A::robot::inertialReg(inertialRegpt);
pros::Rotation _1028A::robot::horizontalencod(horizontalencodpt);
pros::Distance _1028A::robot::ring(ringpt);
pros::Distance _1028A::robot::ringL(ringLpt);
pros::Optical _1028A::robot::optical(opticalpt);

pros::Controller _1028A::robot::master(pros::E_CONTROLLER_MASTER);

lemlib::Drivetrain
    _1028A::robot::drivetrain(&leftMtrs,                  // left motor group
                              &rightMtrs,                 // right motor group
                              11.5,                       // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              2    // horizontal drift is 2 (for now)
    );

lemlib::TrackingWheel
    _1028A::robot::horizontalEncoder(&horizontalencod,
                                     lemlib::Omniwheel::NEW_2, -1);

lemlib::OdomSensors _1028A::robot::sensors(
    nullptr, // vertical tracking wheel 1, set to null
    nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
    &horizontalEncoder, // horizontal tracking wheel 1
    nullptr,  // horizontal tracking wheel 2, set to nullptr as we don't have a
              // second one
    &inertialOdom // inertial sensor
);

lemlib::ControllerSettings _1028A::robot::lateral_controller(
    10,   // proportional gain (kP)
    0,   // integral gain (kI)
    14,   // derivative gain (kD)
    3,   // anti windup
    1,   // small error range, in inches
    100, // small error range timeout, in milliseconds
    3,   // large error range, in inches
    500, // large error range timeout, in milliseconds
    8   // maximum acceleration (slew)
);
lemlib::ControllerSettings _1028A::robot::angular_controller(
    5,   // proportional gain (kP)
    0,   // integral gain (kI)
    27,  // derivative gain (kD)
    0,   // anti windup
    1,   // small error range, in degrees
    100, // small error range timeout, in milliseconds
    3,   // large error range, in degrees
    500, // large error range timeout, in milliseconds
    0    // maximum acceleration (slew)
);

lemlib::ExpoDriveCurve _1028A::robot::throttleCurve(3,10,1.019);
lemlib::ExpoDriveCurve _1028A::robot::turnCurve(3,10,1.019);

lemlib::Chassis _1028A::robot::chassis(
    _1028A::robot::drivetrain,         // drivetrain settings
    _1028A::robot::lateral_controller, // lateral PID settings
    _1028A::robot::angular_controller, // angular PID settings
    _1028A::robot::sensors, 
    &_1028A::robot::throttleCurve,
    &_1028A::robot::turnCurve             // odometry sensors
);
