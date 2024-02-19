#include "1028A/pose.hpp"
#include <math.h>

_1028A::Pose::Pose(float x, float y, float theta) {
  this->x = x;
  this->y = y;
  this->theta = theta;
}

_1028A::Pose _1028A::Pose::operator+(const _1028A::Pose &other) {
  return _1028A::Pose(this->x + other.x, this->y + other.y, this->theta);
}

_1028A::Pose _1028A::Pose::operator-(const _1028A::Pose &other) {
  return _1028A::Pose(this->x - other.x, this->y - other.y, this->theta);
}

float _1028A::Pose::operator*(const _1028A::Pose &other) {
  return this->x * other.x + this->y * other.y;
}

_1028A::Pose _1028A::Pose::operator*(const float &other) {
  return _1028A::Pose(this->x * other, this->y * other, this->theta);
}

_1028A::Pose _1028A::Pose::operator/(const float &other) {
  return _1028A::Pose(this->x / other, this->y / other, this->theta);
}

_1028A::Pose _1028A::Pose::lerp(_1028A::Pose other, float t) {
  return _1028A::Pose(this->x + (other.x - this->x) * t,
                      this->y + (other.y - this->y) * t, this->theta);
}

float _1028A::Pose::distance(_1028A::Pose other) {
  return std::hypot(this->x - other.x, this->y - other.y);
}

float _1028A::Pose::angle(_1028A::Pose other) {
  return std::atan2(other.y - this->y, other.x - this->x);
}

_1028A::Pose _1028A::Pose::rotate(float angle) {
  return _1028A::Pose(this->x * std::cos(angle) - this->y * std::sin(angle),
                      this->x * std::sin(angle) + this->y * std::cos(angle),
                      this->theta);
}
