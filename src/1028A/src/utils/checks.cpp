#include "../src/1028A/include/api.h"
#include "../src/1028A/include/robot.h"
#include "../src/1028A/include/vars.h"

void _1028A::utils::checks::checkBattery() {
  if (pros::battery::get_capacity() < 90) {
    isBatBad = true;
    utils::logger::warn("Battery: low");
  } else {
    isBatBad = false;
  }
}

void _1028A::utils::checks::checkPorts() {
  pros::c::v5_device_e_t LeftFrontcheck =
      pros::c::registry_get_plugged_type((leftFrontPort - 1));
  pros::c::v5_device_e_t LeftMidcheck =
      pros::c::registry_get_plugged_type((leftMidPort - 1));
  pros::c::v5_device_e_t LeftBackcheck =
      pros::c::registry_get_plugged_type((leftBackPort - 1));
  pros::c::v5_device_e_t RightFrontcheck =
      pros::c::registry_get_plugged_type((rightFrontPort - 1));
  pros::c::v5_device_e_t RightMidcheck =
      pros::c::registry_get_plugged_type((rightMidPort - 1));
  pros::c::v5_device_e_t RightBackcheck =
      pros::c::registry_get_plugged_type((rightBackPort - 1));
  pros::c::v5_device_e_t leftAuxcheck =
      pros::c::registry_get_plugged_type((AuxLPort - 1));
  pros::c::v5_device_e_t rightAuxcheck =
      pros::c::registry_get_plugged_type((AuxRPort - 1));
  pros::c::v5_device_e_t Inertialcheck =
      pros::c::registry_get_plugged_type((inertialPort - 1));
  pros::c::v5_device_e_t gpscheck =
      pros::c::registry_get_plugged_type((gpsPort - 1));
  pros::c::v5_device_e_t leftEncodercheck =
      pros::c::registry_get_plugged_type((leftEncoderPort - 1));
  pros::c::v5_device_e_t rightEncodercheck =
      pros::c::registry_get_plugged_type((rightEncoderPort - 1));
  pros::c::v5_device_e_t backEncodercheck =
      pros::c::registry_get_plugged_type((backEncoderPort - 1));

  pros::c::registry_bind_port((leftFrontPort - 1), LeftFrontcheck);
  pros::c::registry_bind_port((leftMidPort - 1), LeftMidcheck);
  pros::c::registry_bind_port((leftBackPort - 1), LeftBackcheck);
  pros::c::registry_bind_port((rightFrontPort - 1), RightFrontcheck);
  pros::c::registry_bind_port((rightMidPort - 1), RightMidcheck);
  pros::c::registry_bind_port((rightBackPort - 1), RightBackcheck);
  pros::c::registry_bind_port((AuxLPort - 1), leftAuxcheck);
  pros::c::registry_bind_port((AuxRPort - 1), rightAuxcheck);
  pros::c::registry_bind_port((inertialPort - 1), Inertialcheck);
  pros::c::registry_bind_port((gpsPort - 1), gpscheck);
  pros::c::registry_bind_port((leftEncoderPort - 1), leftEncodercheck);
  pros::c::registry_bind_port((rightEncoderPort - 1), rightEncodercheck);
  pros::c::registry_bind_port((backEncoderPort - 1), backEncodercheck);

  if (LeftFrontcheck != pros::c::E_DEVICE_MOTOR) {
    utils::logger::error("Left Front Motor Error");
    isPortBad = true;
  }
  if (LeftMidcheck != pros::c::E_DEVICE_MOTOR) {
    utils::logger::error("Left Mid Motor Error");
    isPortBad = true;
  }
  if (LeftBackcheck != pros::c::E_DEVICE_MOTOR) {
    utils::logger::error("Left Back Motor Error");
    isPortBad = true;
  }
  if (RightFrontcheck != pros::c::E_DEVICE_MOTOR) {
    utils::logger::error("Right Front Motor Error");
    isPortBad = true;
  }
  if (RightMidcheck != pros::c::E_DEVICE_MOTOR) {
    utils::logger::error("Right Mid Motor Error");
    isPortBad = true;
  }
  if (RightBackcheck != pros::c::E_DEVICE_MOTOR) {
    utils::logger::error("Right Back Motor Error");
    isPortBad = true;
  }
  if (RightFrontcheck != pros::c::E_DEVICE_MOTOR) {
    utils::logger::error("Right Front Motor Error");
    isPortBad = true;
  }
  if (leftAuxcheck != pros::c::E_DEVICE_MOTOR) {
    utils::logger::error("AuxL Motor Error");
    isPortBad = true;
  }
  if (rightAuxcheck != pros::c::E_DEVICE_MOTOR) {
    utils::logger::error("AuxR Error");
    isPortBad = true;
  }
  if (Inertialcheck != pros::c::E_DEVICE_IMU) {
    utils::logger::error("Inertial Sensor Error");
    isPortBad = true;
  }
  if (gpscheck != pros::c::E_DEVICE_GPS) {
    utils::logger::error("GPS Error");
    isPortBad = true;
  }
  if (leftEncodercheck != pros::c::E_DEVICE_ROTATION) {
    utils::logger::error("Left Encoder Error");
    isPortBad = true;
  }
  if (rightEncodercheck != pros::c::E_DEVICE_ROTATION) {
    utils::logger::error("Right Encoder Error");
    isPortBad = true;
  }
  if (backEncodercheck != pros::c::E_DEVICE_ROTATION) {
    utils::logger::error("Back Encoder Error");
    isPortBad = true;
  }
}

void _1028A::utils::checks::checkMotor() {
  int overTempthreshold = 60;
  isMotorBad = false;
  if (leftFront.get_temperature() >= overTempthreshold) {
    utils::logger::warn("Motor: leftFront overtemp");
    isMotorBad = true;
  }
  if (leftMid.get_temperature() >= overTempthreshold) {
    utils::logger::warn("Motor: leftMid overtemp");
    isMotorBad = true;
  }
  if (leftBack.get_temperature() >= overTempthreshold) {
    utils::logger::warn("Motor: leftBack overtemp");
    isMotorBad = true;
  }
  if (rightFront.get_temperature() >= overTempthreshold) {
    utils::logger::warn("Motor: rightFront overtemp");
    isMotorBad = true;
  }
  if (rightMid.get_temperature() >= overTempthreshold) {
    utils::logger::warn("Motor: rightMid overtemp");
    isMotorBad = true;
  }
  if (rightBack.get_temperature() >= overTempthreshold) {
    utils::logger::warn("Motor: rightBack overtemp");
    isMotorBad = true;
  }
  if (AuxL.get_temperature() >= overTempthreshold) {
    utils::logger::warn("Motor: leftAux overtemp");
    isMotorBad = true;
  }
  if (AuxR.get_temperature() >= overTempthreshold) {
    utils::logger::warn("Motor: rightAux overtemp");
    isMotorBad = true;
  }
}

void _1028A::utils::checks::check() {
  checkBattery();
  checkPorts();
  checkMotor();
}