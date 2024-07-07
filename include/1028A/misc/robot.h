
#include "main.h"

#define leftfrontpt 7
#define leftmidpt 11
#define leftbackpt 12
#define rightfrontpt 10
#define rightmidpt 6
#define rightbackpt 19
#define inertialpt 20
#define hortzencodpt 1

namespace _1028A::robot {
extern pros::Motor leftfront;
extern pros::Motor leftmid;
extern pros::Motor leftback;
extern pros::Motor_Group leftMtrs;
extern pros::Motor rightfront;
extern pros::Motor rightmid;
extern pros::Motor rightback;
extern pros::Motor_Group rightMtrs;
extern pros::IMU inertial;
extern pros::Rotation hortzencod;
extern pros::Controller master;
} // namespace _1028A::robot