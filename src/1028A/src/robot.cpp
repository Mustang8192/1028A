#include "../src/1028A/include/robot.h"

pros::Motor leftFront(leftFrontPort, pros::E_MOTOR_GEARSET_18, false,
                      pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor leftMid(leftMidPort, pros::E_MOTOR_GEARSET_18, false,
                    pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor leftBack(leftBackPort, pros::E_MOTOR_GEARSET_18, false,
                     pros::E_MOTOR_ENCODER_DEGREES);
pros::MotorGroup leftDrive({leftFront, leftMid, leftBack});
pros::Motor rightFront(rightFrontPort, pros::E_MOTOR_GEARSET_18, true,
                       pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor rightMid(rightMidPort, pros::E_MOTOR_GEARSET_18, true,
                     pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor rightBack(rightBackPort, pros::E_MOTOR_GEARSET_18, true,
                      pros::E_MOTOR_ENCODER_DEGREES);
pros::MotorGroup rightDrive({rightFront, rightMid, rightBack});
pros::Motor AuxL(AuxLPort, pros::E_MOTOR_GEARSET_18, false,
                 pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor AuxR(AuxRPort, pros::E_MOTOR_GEARSET_18, true,
                 pros::E_MOTOR_ENCODER_DEGREES);
pros::MotorGroup Aux({AuxL, AuxR});
pros::Imu inertial(inertialPort);
pros::GPS gps(gpsPort);
pros::Rotation leftEncoder(leftEncoderPort);
pros::Rotation rightEncoder(rightEncoderPort);
pros::Rotation backEncoder(backEncoderPort);
pros::Controller mainController(pros::E_CONTROLLER_MASTER);
