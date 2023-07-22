#include "../src/1028A/include/api.h"

/**
 * @brief Create a new pose
 *
 * @param x component
 * @param y component
 * @param theta heading. Defaults to 0
 */
_1028A::odom::Pose::Pose(float x, float y, float theta) {
  this->x = x;
  this->y = y;
  this->theta = theta;
}

/**
 * @brief Add a pose to this pose
 *
 * @param other other pose
 * @return Pose
 */
_1028A::odom::Pose
_1028A::odom::Pose::operator+(const _1028A::odom::Pose &other) {
  return _1028A::odom::Pose(this->x + other.x, this->y + other.y, this->theta);
}

/**
 * @brief Subtract a pose from this pose
 *
 * @param other other pose
 * @return Pose
 */
_1028A::odom::Pose
_1028A::odom::Pose::operator-(const _1028A::odom::Pose &other) {
  return _1028A::odom::Pose(this->x - other.x, this->y - other.y, this->theta);
}

/**
 * @brief Multiply a pose by this pose
 *
 * @param other other pose
 * @return Pose
 */
float _1028A::odom::Pose::operator*(const _1028A::odom::Pose &other) {
  return this->x * other.x + this->y * other.y;
}

/**
 * @brief Multiply a pose by a float
 *
 * @param other float
 * @return Pose
 */
_1028A::odom::Pose _1028A::odom::Pose::operator*(const float &other) {
  return _1028A::odom::Pose(this->x * other, this->y * other, this->theta);
}

/**
 * @brief Divide a pose by a float
 *
 * @param other float
 * @return Pose
 */
_1028A::odom::Pose _1028A::odom::Pose::operator/(const float &other) {
  return _1028A::odom::Pose(this->x / other, this->y / other, this->theta);
}

/**
 * @brief Linearly interpolate between two poses
 *
 * @param other the other pose
 * @param t t value
 * @return Pose
 */
_1028A::odom::Pose _1028A::odom::Pose::lerp(_1028A::odom::Pose other, float t) {
  return _1028A::odom::Pose(this->x + (other.x - this->x) * t,
                            this->y + (other.y - this->y) * t, this->theta);
}

/**
 * @brief Get the distance between two poses
 *
 * @param other the other pose
 * @return float
 */
float _1028A::odom::Pose::distance(_1028A::odom::Pose other) {
  return std::hypot(this->x - other.x, this->y - other.y);
}

/**
 * @brief Get the angle between two poses
 *
 * @param other the other pose
 * @return float in radians
 */
float _1028A::odom::Pose::angle(_1028A::odom::Pose other) {
  return std::atan2(other.y - this->y, other.x - this->x);
}

/**
 * @brief Rotate a pose by an angle
 *
 * @param angle angle in radians
 * @return Pose
 */
_1028A::odom::Pose _1028A::odom::Pose::rotate(float angle) {
  return _1028A::odom::Pose(
      this->x * std::cos(angle) - this->y * std::sin(angle),
      this->x * std::sin(angle) + this->y * std::cos(angle), this->theta);
}
