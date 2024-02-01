#include "1028A/robot.h"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "main.h"
#include "pros/adi.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"

pros::Motor _1028A::robot::leftfront(leftfrontpt, pros::E_MOTOR_GEARSET_18,
                                     true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor _1028A::robot::leftmid(leftmidpt, pros::E_MOTOR_GEARSET_18, true,
                                   pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor _1028A::robot::leftback(leftbackpt, pros::E_MOTOR_GEARSET_18, false,
                                    pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor_Group _1028A::robot::leftMtrs({leftfrontpt, leftmidpt, leftbackpt});

pros::Motor _1028A::robot::rightfront(rightfrontpt, pros::E_MOTOR_GEARSET_18,
                                      false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor _1028A::robot::rightmid(rightmidpt, pros::E_MOTOR_GEARSET_18, false,
                                    pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor _1028A::robot::rightback(rightbackpt, pros::E_MOTOR_GEARSET_18,
                                     true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor_Group _1028A::robot::rightMtrs({rightfrontpt, rightmidpt,
                                            rightbackpt});

pros::Motor _1028A::robot::intake(inakept, pros::E_MOTOR_GEARSET_18, false,
                                  pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor _1028A::robot::kicker(kickerpt, pros::E_MOTOR_GEARSET_18, false,
                                  pros::E_MOTOR_ENCODER_DEGREES);
pros::IMU _1028A::robot::inertial(inertialpt);
pros::Rotation _1028A::robot::leftEnc(leftencpt);
pros::Rotation _1028A::robot::rightEnc(rightencpt);
pros::ADIDigitalOut _1028A::robot::flapL('A');
pros::ADIDigitalOut _1028A::robot::flapR('C');
pros::ADIDigitalOut _1028A::robot::climb('B');
pros::ADIDigitalOut _1028A::robot::stick('D');

pros::Controller _1028A::robot::master(pros::E_CONTROLLER_MASTER);

lemlib::Drivetrain_t _1028A::robot::drivetrain{
    &leftMtrs,                  // left drivetrain motors
    &rightMtrs,                 // right drivetrain motors
    10,                         // track width
    lemlib::Omniwheel::NEW_325, // wheel diameter
    600                         // wheel rpm
};

lemlib::TrackingWheel
    _1028A::robot::leftTracker(&leftEnc, lemlib::Omniwheel::NEW_275, -4.6);
lemlib::TrackingWheel
    _1028A::robot::rightTracker(&rightEnc, lemlib::Omniwheel::NEW_275, 0);

lemlib::OdomSensors_t _1028A::robot::odomSensors{
    &leftTracker, // vertical tracking wheel
    &rightTracker,
    nullptr, // horizontal tracking wheel
    nullptr,
    &inertial // inertial sensor
};

lemlib::ChassisController_t _1028A::robot::lateralController{
    8,   // kP
    30,  // kD
    1,   // smallErrorRange
    100, // smallErrorTimeout
    3,   // largeErrorRange
    500, // largeErrorTimeout
    5    // slew rate
};

lemlib::ChassisController_t _1028A::robot::angularController{
    4,   // kP
    40,  // kD
    1,   // smallErrorRange
    100, // smallErrorTimeout
    3,   // largeErrorRange
    500, // largeErrorTimeout
    40   // slew rate
};

lemlib::Chassis _1028A::robot::chassis(drivetrain, lateralController,
                                       angularController, odomSensors);