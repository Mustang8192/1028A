#include "1028A/util.hpp"
#include "1028A/pose.hpp"
#include <math.h>
#include <vector>

float _1028A::slew(float target, float current, float maxChange) {
  float change = target - current;
  if (maxChange == 0)
    return target;
  if (change > maxChange)
    change = maxChange;
  else if (change < -maxChange)
    change = -maxChange;
  return current + change;
}

float _1028A::radToDeg(float rad) { return rad * 180 / M_PI; }

float _1028A::degToRad(float deg) { return deg * M_PI / 180; }

float _1028A::angleError(float angle1, float angle2, bool radians) {
  float max = radians ? 2 * M_PI : 360;
  float half = radians ? M_PI : 180;
  angle1 = fmod(angle1, max);
  angle2 = fmod(angle2, max);
  float error = angle1 - angle2;
  if (error > half)
    error -= max;
  else if (error < -half)
    error += max;
  return error;
}

int _1028A::sgn(float x) {
  if (x < 0)
    return -1;
  else
    return 1;
}

float _1028A::avg(std::vector<float> values) {
  float sum = 0;
  for (float value : values) {
    sum += value;
  }
  return sum / values.size();
}

float _1028A::ema(float current, float previous, float smooth) {
  return (current * smooth) + (previous * (1 - smooth));
}

float _1028A::getCurvature(Pose pose, Pose other) {
  // calculate whether the pose is on the left or right side of the circle
  float side = _1028A::sgn(std::sin(pose.theta) * (other.x - pose.x) -
                           std::cos(pose.theta) * (other.y - pose.y));
  // calculate center point and radius
  float a = -std::tan(pose.theta);
  float c = std::tan(pose.theta) * pose.x - pose.y;
  float x = std::fabs(a * other.x + other.y + c) / std::sqrt((a * a) + 1);
  float d = std::hypot(other.x - pose.x, other.y - pose.y);

  // return curvature
  return side * ((2 * x) / (d * d));
}
