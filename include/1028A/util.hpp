

#pragma once

#include "1028A/pose.hpp"
#include <vector>

namespace _1028A {
float slew(float target, float current, float maxChange);

float radToDeg(float rad);

float degToRad(float deg);

float angleError(float angle1, float angle2, bool radians = false);

int sgn(float x);

float avg(std::vector<float> values);

float ema(float current, float previous, float smooth);

float getCurvature(Pose pose, Pose other);
} // namespace _1028A