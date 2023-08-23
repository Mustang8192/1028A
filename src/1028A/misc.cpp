#include "1028A/misc.h"
#include "../1028A/addons/gif/gifclass.hpp"
#include "1028A/logger.h"
#include "1028A/robot.h"
#include "1028A/vars.h"

/*omit
 * @brief Slew rate limiter
 *
 * @param target target value
 * @param current current value
 * @param maxChange maximum change. No maximum if set to 0
 * @return float - the limited value
 */
float _1028A::utils::slew(float target, float current, float maxChange) {
  float change = target - current;
  if (maxChange == 0)
    return target;
  if (change > maxChange)
    change = maxChange;
  else if (change < -maxChange)
    change = -maxChange;
  return current + change;
}

/*omit
 * @brief Convert radians to degrees
 *
 * @param rad radians
 * @return float degrees
 */
float _1028A::utils::radToDeg(float rad) { return rad * 180 / M_PI; }

/*omit
 * @brief Convert degrees to radians
 *
 * @param deg degrees
 * @return float radians
 */
float _1028A::utils::degToRad(float deg) { return deg * M_PI / 180; }

/*omit
 * @brief Calculate the error between 2 angles. Useful when calculating the
 * error between 2 headings
 *
 * @param angle1
 * @param angle2
 * @param radians true if angle is in radians, false if not. False by default
 * @return float wrapped angle
 */
float _1028A::utils::angleError(float angle1, float angle2, bool radians) {
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

/*omit
 * @brief Return the sign of a number
 *
 * @param x the number to get the sign of
 * @return float - -1 if negative, 1 if positive
 */
float _1028A::utils::sgn(float x) {
  if (x < 0)
    return -1;
  else
    return 1;
}

double _1028A::utils::tickToFt(double ticks) {
  return (ticks / 300) * (3.0 / 5) * (3.25 * M_PI) / 12;
}

double _1028A::utils::rpmToFtps(double rpm) {
  return rpm / 60 * (3.0 / 5) * (3.25 * M_PI) / 12;
}

double _1028A::utils::ftToMeters(double ft) { return ft * 0.3048; }

double _1028A::utils::metersToFt(double meters) { return meters * 3.28084; }
/*omit
 * @brief Return the average of a vector of numbers
 *
 * @param values
 * @return float
 */
float _1028A::utils::avg(std::vector<float> values) {
  float sum = 0;
  for (float value : values) {
    sum += value;
  }
  return sum / values.size();
}

float _1028A::utils::reduce_0_to_360(float angle) {
  while (!(angle >= 0 && angle < 360)) {
    if (angle < 0) {
      angle += 360;
    }
    if (angle >= 360) {
      angle -= 360;
    }
  }
  return (angle);
}

float _1028A::utils::reduce_negative_180_to_180(float angle) {
  while (!(angle >= -180 && angle < 180)) {
    if (angle < -180) {
      angle += 360;
    }
    if (angle >= 180) {
      angle -= 360;
    }
  }
  return (angle);
}

float _1028A::utils::reduce_negative_90_to_90(float angle) {
  while (!(angle >= -90 && angle < 90)) {
    if (angle < -90) {
      angle += 180;
    }
    if (angle >= 90) {
      angle -= 180;
    }
  }
  return (angle);
}

void _1028A::utils::checks() {
  pros::c::v5_device_e_t LeftFrontcheck =
      pros::c::registry_get_plugged_type((leftfrontpt - 1));
  pros::c::v5_device_e_t LeftMidcheck =
      pros::c::registry_get_plugged_type((leftmidpt - 1));
  pros::c::v5_device_e_t LeftBackcheck =
      pros::c::registry_get_plugged_type((leftbackpt - 1));
  pros::c::v5_device_e_t RightFrontcheck =
      pros::c::registry_get_plugged_type((rightfrontpt - 1));
  pros::c::v5_device_e_t RightMidcheck =
      pros::c::registry_get_plugged_type((rightmidpt - 1));
  pros::c::v5_device_e_t RightBackcheck =
      pros::c::registry_get_plugged_type((rightbackpt - 1));
  pros::c::v5_device_e_t AuxL11check =
      pros::c::registry_get_plugged_type((auxL11pt - 1));
  pros::c::v5_device_e_t AuxL55check =
      pros::c::registry_get_plugged_type((auxL55pt - 1));
  pros::c::v5_device_e_t AuxR11check =
      pros::c::registry_get_plugged_type((auxR11pt - 1));
  pros::c::v5_device_e_t AuxR55check =
      pros::c::registry_get_plugged_type((auxR55pt - 1));
  pros::c::v5_device_e_t VerticalEnccheck =
      pros::c::registry_get_plugged_type((verticalEncpt - 1));
  pros::c::v5_device_e_t HorizontalEnccheck =
      pros::c::registry_get_plugged_type((horizontalEncpt - 1));
  pros::c::v5_device_e_t Inertialcheck =
      pros::c::registry_get_plugged_type((inertialpt - 1));
  pros::c::v5_device_e_t InertialOdomcheck =
      pros::c::registry_get_plugged_type((inertialOdompt - 1));
  if (LeftFrontcheck != pros::c::E_DEVICE_MOTOR) {
    _1028A::logger::fatal("Left Front Motor not found");
    ports = false;
  }
  if (LeftMidcheck != pros::c::E_DEVICE_MOTOR) {
    _1028A::logger::fatal("Left Mid Motor not found");
    ports = false;
  }
  if (LeftBackcheck != pros::c::E_DEVICE_MOTOR) {
    _1028A::logger::fatal("Left Back Motor not found");
    ports = false;
  }
  if (RightFrontcheck != pros::c::E_DEVICE_MOTOR) {
    _1028A::logger::fatal("Right Front Motor not found");
    ports = false;
  }
  if (RightMidcheck != pros::c::E_DEVICE_MOTOR) {
    _1028A::logger::fatal("Right Mid Motor not found");
    ports = false;
  }
  if (RightBackcheck != pros::c::E_DEVICE_MOTOR) {
    _1028A::logger::fatal("Right Back Motor not found");
    ports = false;
  }
  if (AuxL11check != pros::c::E_DEVICE_MOTOR) {
    _1028A::logger::fatal("Aux L11 Motor not found");
    ports = false;
  }
  if (AuxL55check != pros::c::E_DEVICE_MOTOR) {
    _1028A::logger::fatal("Aux L55 Motor not found");
    ports = false;
  }
  if (AuxR11check != pros::c::E_DEVICE_MOTOR) {
    _1028A::logger::fatal("Aux R11 Motor not found");
    ports = false;
  }
  if (AuxR55check != pros::c::E_DEVICE_MOTOR) {
    _1028A::logger::fatal("Aux R55 Motor not found");
    ports = false;
  }
  if (VerticalEnccheck != pros::c::E_DEVICE_ROTATION) {
    _1028A::logger::fatal("Vertical Encoder not found");
    ports = false;
  }
  if (HorizontalEnccheck != pros::c::E_DEVICE_ROTATION) {
    _1028A::logger::fatal("Horizontal Encoder not found");
    ports = false;
  }
  if (Inertialcheck != pros::c::E_DEVICE_IMU) {
    _1028A::logger::fatal("Inertial Sensor not found");
    ports = false;
  }
  if (InertialOdomcheck != pros::c::E_DEVICE_IMU) {
    _1028A::logger::fatal("Inertial Odom Sensor not found");
    ports = false;
  }

  if (_1028A::robot::leftfront.is_over_temp()) {
    _1028A::logger::fatal("Left Front Motor overheating");
    overTemp = true;
  }
  if (_1028A::robot::leftmid.is_over_temp()) {
    _1028A::logger::fatal("Left Mid Motor overheating");
    overTemp = true;
  }
  if (_1028A::robot::leftback.is_over_temp()) {
    _1028A::logger::fatal("Left Back Motor overheating");
    overTemp = true;
  }
  if (_1028A::robot::rightfront.is_over_temp()) {
    _1028A::logger::fatal("Right Front Motor overheating");
    overTemp = true;
  }
  if (_1028A::robot::rightmid.is_over_temp()) {
    _1028A::logger::fatal("Right Mid Motor overheating");
    overTemp = true;
  }
  if (_1028A::robot::rightback.is_over_temp()) {
    _1028A::logger::fatal("Right Back Motor overheating");
    overTemp = true;
  }
  if (_1028A::robot::auxL11.is_over_temp()) {
    _1028A::logger::fatal("Aux L11 Motor overheating");
    overTemp = true;
  }
  if (_1028A::robot::auxL55.is_over_temp()) {
    _1028A::logger::fatal("Aux L55 Motor overheating");
    overTemp = true;
  }
  if (_1028A::robot::auxR11.is_over_temp()) {
    _1028A::logger::fatal("Aux R11 Motor overheating");
    overTemp = true;
  }
  if (_1028A::robot::auxR55.is_over_temp()) {
    _1028A::logger::fatal("Aux R55 Motor overheating");
    overTemp = true;
  }

  if (pros::battery::get_capacity() < 90) {
    batteryLow = true;
    _1028A::logger::warn("Battery: low");
  }
}
void _1028A::utils::init() {
  static Gif gif("/usd/Ui/intro.gif", lv_scr_act());
  pros::delay(11400);
  gif.pause();
  gif.clean();
  _1028A::logger::init();
  _1028A::utils::checks();
}
