#include "1028A/misc/robot.h"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include <cstddef>

#define trackwidth 9
#define driveRPM 600
#define vertTrackingOffset -1.5
#define horizontalTrackingOffset -1

pros::Motor _1028A::robot::leftfront(leftFrontPort, pros::E_MOTOR_GEARSET_06,
                                     true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor _1028A::robot::leftmid(leftMidPort, pros::E_MOTOR_GEARSET_06, true,
                                   pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor _1028A::robot::leftback(leftBackPort, pros::E_MOTOR_GEARSET_06, true,
                                    pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor_Group _1028A::robot::leftMtrs({leftFrontPort, leftMidPort, leftBackPort});

pros::Motor _1028A::robot::rightfront(rightFrontPort, pros::E_MOTOR_GEARSET_06,
                                      false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor _1028A::robot::rightmid(rightMidPort, pros::E_MOTOR_GEARSET_06, false,
                                    pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor _1028A::robot::rightback(rightBackPort, pros::E_MOTOR_GEARSET_06,
                                     false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor_Group _1028A::robot::rightMtrs({rightFrontPort, rightMidPort,
                                            rightBackPort});
pros::Motor _1028A::robot::intake(intakePort, pros::E_MOTOR_GEARSET_06, true,
                                  pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor _1028A::robot::LBL (LBLPort);
pros::Motor _1028A::robot::LBR (LBRPort);
pros::MotorGroup _1028A::robot::LB({-LBLPort, LBRPort});
pros::Rotation _1028A::robot::LBS(LBSPort);

pros::ADIDigitalOut _1028A::robot::mogo('h');
pros::IMU _1028A::robot::inertial(inertialPort);
pros::Rotation _1028A::robot::horizontalencod(horizontalPort);
pros::Rotation _1028A::robot::verticalencod(VerticalPort);

pros::Controller _1028A::robot::master(pros::E_CONTROLLER_MASTER);

lemlib::Drivetrain _1028A::robot::drivetrain(&leftMtrs, &rightMtrs, trackwidth, lemlib::Omniwheel::OLD_275, driveRPM, 2);
lemlib::TrackingWheel _1028A::robot::vertTracking(&verticalencod, lemlib::Omniwheel::NEW_2, vertTrackingOffset);
lemlib::TrackingWheel _1028A::robot::horizontalTracking(&horizontalencod, lemlib::Omniwheel::NEW_2, horizontalTrackingOffset);
lemlib::OdomSensors _1028A::robot::sensors(&vertTracking, nullptr, &horizontalTracking, nullptr, &inertial);
lemlib::ControllerSettings _1028A::robot::lateralController(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);
lemlib::ControllerSettings _1028A::robot::angularController(3, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

lemlib::ExpoDriveCurve _1028A::robot::driveCurve(3,10,1.019);
lemlib::ExpoDriveCurve _1028A::robot::turnCurve(3,10,1.019);

lemlib::Chassis  _1028A::robot::chassis(
    _1028A::robot::drivetrain,
    _1028A::robot::lateralController,
    _1028A::robot::angularController,
    _1028A::robot::sensors, 
    &_1028A::robot::driveCurve,
    &_1028A::robot::turnCurve 
);