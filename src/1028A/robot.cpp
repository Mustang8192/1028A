#include "1028A/robot.h"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "mcl.h"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"

pros::Motor _1028A::robot::leftFront(leftFrontpt);
pros::Motor _1028A::robot::leftMid(leftMidpt);
pros::Motor _1028A::robot::leftBack(leftBackpt);
pros::MotorGroup _1028A::robot::leftMtrs({-leftFrontpt, -leftMidpt, -leftBackpt});
pros::Motor _1028A::robot::rightFront(rightFrontpt);
pros::Motor _1028A::robot::rightMid(rightMidpt);
pros::Motor _1028A::robot::rightBack(rightBackpt);
pros::MotorGroup _1028A::robot::rightMtrs({rightFrontpt, rightMidpt, rightBackpt});
pros::Motor _1028A::robot::intake(intakept);
pros::Motor _1028A::robot::LBL(LBLPort);
pros::Motor _1028A::robot::LBR(LBRPort);
pros::MotorGroup _1028A::robot::LB({LBLPort, -LBRPort});
pros::Rotation _1028A::robot::LBS(LBSPort);
pros::adi::DigitalIn _1028A::robot::LBSLimit('f');
pros::adi::DigitalIn  _1028A::robot::CaliSwitch('h');
pros::Rotation _1028A::robot::Vertical(-Verticalpt);
pros::Rotation _1028A::robot::Horizontal(-Horizontalpt);
pros::Imu _1028A::robot::inertial(inertialpt);
pros::Distance _1028A::robot::distance(distancePort);
pros::Distance _1028A::robot::Ldistance(LdistancePort);
pros::Optical _1028A::robot::optical(opticalPort);
pros::Optical _1028A::robot::opticalH(opticalPortH);
pros::adi::DigitalOut _1028A::robot::mogo(mogoPort);
pros::adi::DigitalOut _1028A::robot::stickR('e');
pros::adi::DigitalOut _1028A::robot::stickL('f');
pros::Controller _1028A::robot::master(pros::E_CONTROLLER_MASTER);

//_1028A::MonteCarloLocalizer _1028A::robot::localizer(5,6,2,4,getX, getY, getTheta, 144, 144, 300);


lemlib::Drivetrain _1028A::robot::drivetrain(&leftMtrs, &rightMtrs, 10, lemlib::Omniwheel::OLD_325, 450, 2);
lemlib::TrackingWheel _1028A::robot::vertical(&Vertical, 1.99, -1.5);
lemlib::TrackingWheel _1028A::robot::horizontal(&Horizontal, 1.99, -1.5);
lemlib::OdomSensors _1028A::robot::sensors(&vertical, nullptr, &horizontal, nullptr, &inertial);
lemlib::ControllerSettings _1028A::robot::lateralController(8, 0, 27, 4, 1, 100, 3, 500, 20);
lemlib::ControllerSettings _1028A::robot::angularController(1.7, 0, 5, 3,0.5, 100, 2, 500, 0);
lemlib::ExpoDriveCurve _1028A::robot::driveCurve(3, 10, 1.019);
lemlib::ExpoDriveCurve _1028A::robot::turnCurve(3, 10, 1.019);
lemlib::Chassis _1028A::robot::chassis(_1028A::robot::drivetrain, _1028A::robot::lateralController, _1028A::robot::angularController, _1028A::robot::sensors, &_1028A::robot::driveCurve, &_1028A::robot::turnCurve);
