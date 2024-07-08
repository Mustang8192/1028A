
#include "main.h"

#define leftfrontpt 1
#define leftmidpt 2
#define leftbackpt 3
#define rightfrontpt 10
#define rightmidpt 9
#define rightbackpt 8
#define intakept 5
#define conveyorpt 7
#define inertialpt 20
#define hortzencodpt 19

namespace _1028A::robot {
extern pros::Motor leftfront;
extern pros::Motor leftmid;
extern pros::Motor leftback;
extern pros::Motor_Group leftMtrs;
extern pros::Motor rightfront;
extern pros::Motor rightmid;
extern pros::Motor rightback;
extern pros::Motor_Group rightMtrs;
extern pros::Motor intake;
extern pros::Motor conveyor;
extern pros::Motor_Group intakeMtrs;
extern pros::ADIDigitalOut mogo;
extern pros::IMU inertial;
extern pros::Rotation hortzencod;
extern pros::Controller master;
} // namespace _1028A::robot