
#pragma once

namespace _1028A {
class Pose {
public:
  float x;
  float y;
  float theta;
  Pose(float x, float y, float theta = 0);
  Pose operator+(const Pose &other);
  Pose operator-(const Pose &other);
  float operator*(const Pose &other);
  Pose operator*(const float &other);
  Pose operator/(const float &other);
  Pose lerp(Pose other, float t);
  float distance(Pose other);
  float angle(Pose other);
  Pose rotate(float angle);
};
} // namespace _1028A
