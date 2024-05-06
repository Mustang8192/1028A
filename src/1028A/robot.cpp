#include "1028A/robot.h"

pros::Motor _1028A::robot::leftfront(leftfrontpt, pros::E_MOTOR_GEARSET_18,
                                     true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor _1028A::robot::leftmid(leftmidpt, pros::E_MOTOR_GEARSET_18, true,
                                   pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor _1028A::robot::leftback(leftbackpt, pros::E_MOTOR_GEARSET_18, true,
                                    pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor_Group _1028A::robot::leftMtrs({leftfrontpt, leftmidpt, leftbackpt});

pros::Motor _1028A::robot::rightfront(rightfrontpt, pros::E_MOTOR_GEARSET_18,
                                      false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor _1028A::robot::rightmid(rightmidpt, pros::E_MOTOR_GEARSET_18, false,
                                    pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor _1028A::robot::rightback(rightbackpt, pros::E_MOTOR_GEARSET_18,
                                     false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor_Group _1028A::robot::rightMtrs({rightfrontpt, rightmidpt,
                                            rightbackpt});

pros::Motor _1028A::robot::intake(inakept, pros::E_MOTOR_GEARSET_18, false,
                                  pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor _1028A::robot::kicker(kickerpt, pros::E_MOTOR_GEARSET_18, false,
                                  pros::E_MOTOR_ENCODER_DEGREES);
pros::IMU _1028A::robot::inertial(inertialpt);
pros::Rotation _1028A::robot::leftEnc(leftencpt);
pros::Rotation _1028A::robot::rightEnc(rightencpt);

pros::ADIDigitalOut _1028A::robot::flapL(flapLpt);
pros::ADIDigitalOut _1028A::robot::flapR(flapRpt);
pros::ADIDigitalOut _1028A::robot::climb_set1(climb_set1pt);
pros::ADIDigitalOut _1028A::robot::climb_set2(climb_set2pt);
pros::ADIDigitalOut _1028A::robot::backL(backLpt);
pros::ADIDigitalOut _1028A::robot::backR(backRpt);
pros::ADIDigitalIn _1028A::robot::limitSwitch(limitSwitchpt);

pros::Controller _1028A::robot::master(pros::E_CONTROLLER_MASTER);
