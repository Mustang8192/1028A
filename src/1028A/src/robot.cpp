#include "../src/1028A/include/robot.h"

pros::Motor leftFront(leftFrontPort, pros::E_MOTOR_GEARSET_18, false,
                      pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor leftMid(leftMidPort, pros::E_MOTOR_GEARSET_18, false,
                    pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor leftBack(leftBackPort, pros::E_MOTOR_GEARSET_18, false,
                     pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor_Group leftDrive({leftFront, leftMid, leftBack});
pros::Motor rightFront(rightFrontPort, pros::E_MOTOR_GEARSET_18, true,
                       pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor rightMid(rightMidPort, pros::E_MOTOR_GEARSET_18, true,
                     pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor rightBack(rightBackPort, pros::E_MOTOR_GEARSET_18, true,
                      pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor_Group rightDrive({rightFront, rightMid, rightBack});
pros::Motor AuxL(AuxLPort, pros::E_MOTOR_GEARSET_18, false,
                 pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor AuxR(AuxRPort, pros::E_MOTOR_GEARSET_18, true,
                 pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor_Group Aux({AuxL, AuxR});
pros::Imu inertial(inertialPort);
pros::GPS gps(gpsPort);
pros::Rotation leftEncoder(leftEncoderPort);
pros::Rotation rightEncoder(rightEncoderPort);
pros::Rotation backEncoder(backEncoderPort);
pros::Controller mainController(pros::E_CONTROLLER_MASTER);

_1028A::TrackingWheel leftTracker(&leftEncoder, 2.75, 10);
_1028A::TrackingWheel rightTracker(&rightEncoder, 2.75, 10);
_1028A::TrackingWheel backTracker(&backEncoder, 2.75, 10);

_1028A::Drivetrain_t drivetrain = {&leftDrive, &rightDrive, 10, 2.75, 600};
_1028A::OdomSensors_t OdomSensors = {&leftTracker, &rightTracker, &backTracker,
                                     nullptr, &inertial};
// forward/backward PID
_1028A::ChassisController_t lateralController{
    8,   // kP
    30,  // kD
    1,   // smallErrorRange
    100, // smallErrorTimeout
    3,   // largeErrorRange
    500, // largeErrorTimeout
    5    // slew rate
};

// turning PID
_1028A::ChassisController_t angularController{
    4,   // kP
    40,  // kD
    1,   // smallErrorRange
    100, // smallErrorTimeout
    3,   // largeErrorRange
    500, // largeErrorTimeout
    40   // slew rate
};

_1028A::Chassis chassis(drivetrain, lateralController, angularController,
                        OdomSensors);
