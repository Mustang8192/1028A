

#pragma once

#include "1028A/chassis/chassis.hpp"
#include "1028A/pose.hpp"

namespace _1028A {
void setSensors(_1028A::OdomSensors_t sensors, _1028A::Drivetrain_t drivetrain);
Pose getPose(bool radians = false);
void setPose(Pose pose, bool radians = false);
Pose getSpeed(bool radians = false);
Pose getLocalSpeed(bool radians = false);
Pose estimatePose(float time, bool radians = false);
void update();
void init();
} // namespace _1028A
