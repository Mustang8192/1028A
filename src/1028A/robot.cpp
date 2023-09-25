#include "1028A/robot.h"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "main.h"
#include "pros/motors.hpp"
#include "pros/optical.hpp"

pros::Motor _1028A::robot::leftfront(leftfrontpt, pros::E_MOTOR_GEARSET_18,
                                     false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor _1028A::robot::leftmid(leftmidpt, pros::E_MOTOR_GEARSET_18, false,
                                   pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor _1028A::robot::leftback(leftbackpt, pros::E_MOTOR_GEARSET_18, false,
                                    pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor_Group _1028A::robot::leftMtrs({leftfrontpt, leftmidpt, leftbackpt});

pros::Motor_Group _1028A::robot::leftMtrswPTO({leftfrontpt, leftmidpt,
                                               leftbackpt, auxL11pt, auxL55pt});
pros::Motor _1028A::robot::rightfront(rightfrontpt, pros::E_MOTOR_GEARSET_18,
                                      true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor _1028A::robot::rightmid(rightmidpt, pros::E_MOTOR_GEARSET_18, true,
                                    pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor _1028A::robot::rightback(rightbackpt, pros::E_MOTOR_GEARSET_18,
                                     true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor_Group _1028A::robot::rightMtrs({rightfrontpt, rightmidpt,
                                            rightbackpt});
pros::Motor_Group _1028A::robot::rightMtrswPTO({rightfrontpt, rightmidpt,
                                                rightbackpt, auxR11pt,
                                                auxR55pt});
pros::Motor _1028A::robot::auxL11(auxL11pt, pros::E_MOTOR_GEARSET_18, false,
                                  pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor _1028A::robot::auxL55(auxL55pt, pros::E_MOTOR_GEARSET_18, false,
                                  pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor _1028A::robot::auxR11(auxR11pt, pros::E_MOTOR_GEARSET_18, true,
                                  pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor _1028A::robot::auxR55(auxR55pt, pros::E_MOTOR_GEARSET_18, true,
                                  pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor _1028A::robot::intake(inakept, pros::E_MOTOR_GEARSET_18, false,
                                  pros::E_MOTOR_ENCODER_DEGREES);
pros::Rotation _1028A::robot::verticalEnc(verticalEncpt);
pros::Rotation _1028A::robot::horizontalEnc(horizontalEncpt);
pros::IMU _1028A::robot::inertial(inertialpt);
pros::IMU _1028A::robot::inertialOdom(inertialOdompt);
pros::GPS _1028A::robot::gps1(gps1pt);
pros::GPS _1028A::robot::gps2(gps2pt);
pros::Optical _1028A::robot::optical(opticalpt);
_1028A::GPSRedundantSensor _1028A::robot::gps(&_1028A::robot::gps1,
                                              &_1028A::robot::gps2, 1);

pros::Controller _1028A::robot::master(pros::E_CONTROLLER_MASTER);

lemlib::Drivetrain_t _1028A::robot::drivetrain{
    &leftMtrswPTO,              // left drivetrain motors
    &rightMtrswPTO,             // right drivetrain motors
    10,                         // track width
    lemlib::Omniwheel::NEW_275, // wheel diameter
    600                         // wheel rpm
};

lemlib::TrackingWheel _1028A::robot::verticalTracker(&verticalEnc,
                                                     lemlib::Omniwheel::NEW_275,
                                                     -4.6);
lemlib::TrackingWheel
    _1028A::robot::horizontalTracker(&horizontalEnc, lemlib::Omniwheel::NEW_275,
                                     0);

lemlib::OdomSensors_t _1028A::robot::odomSensors{
    &verticalTracker, // vertical tracking wheel
    nullptr,
    &horizontalTracker, // horizontal tracking wheel
    nullptr,
    &inertialOdom // inertial sensor
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