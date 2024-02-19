

#pragma once

#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"

namespace _1028A {

namespace Omniwheel {
constexpr float NEW_275 = 2.75;
constexpr float OLD_275 = 2.75;
constexpr float NEW_275_HALF = 2.744;
constexpr float OLD_275_HALF = 2.74;
constexpr float NEW_325 = 3.25;
constexpr float OLD_325 = 3.25;
constexpr float NEW_325_HALF = 3.246;
constexpr float OLD_325_HALF = 3.246;
constexpr float NEW_4 = 4;
constexpr float OLD_4 = 4.18;
constexpr float NEW_4_HALF = 3.995;
constexpr float OLD_4_HALF = 4.175;
} // namespace Omniwheel

class TrackingWheel {
public:
  TrackingWheel(pros::ADIEncoder *encoder, float wheelDiameter, float distance,
                float gearRatio = 1);
  TrackingWheel(pros::Rotation *encoder, float wheelDiameter, float distance,
                float gearRatio = 1);
  TrackingWheel(pros::Motor_Group *motors, float wheelDiameter, float distance,
                float rpm);
  void reset();
  float getDistanceTraveled();
  float getOffset();
  int getType();

private:
  float diameter;
  float distance;
  float rpm;
  pros::ADIEncoder *encoder = nullptr;
  pros::Rotation *rotation = nullptr;
  pros::Motor_Group *motors = nullptr;
  float gearRatio = 1;
};
} // namespace _1028A