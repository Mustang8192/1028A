#include "1028A/sensors.h"

_1028A::GPSRedundantSensor::GPSRedundantSensor(pros::GPS *s1, pros::GPS *s2,
                                               double maxDiff)
    : sensor1(s1), sensor2(s2), maxDifference(maxDiff) {}

double _1028A::GPSRedundantSensor::getXCoordinate() {
  pros::c::gps_status_s_t data1, data2;
  data1 = sensor1->get_status();
  data2 = sensor2->get_status();
  double result;
  double difference = fabs(data1.x - data2.x);

  if (difference <= maxDifference) {
    result = (data1.x + data2.x) / 2.0;
  } else if (fabs(data1.x) < fabs(data2.x)) {
    result = data1.x;
  } else if (fabs(data1.x) > fabs(data2.x)) {
    result = data2.x;
  } else {
    result = NAN; // Or any other value to indicate unreliable data
  }
  return result;
}

double _1028A::GPSRedundantSensor::getYCoordinate() {
  pros::c::gps_status_s_t data1, data2;
  data1 = sensor1->get_status();
  data2 = sensor2->get_status();
  double result;
  double difference = fabs(data1.y - data2.y);

  if (difference <= maxDifference) {
    result = (data1.y + data2.y) / 2.0;
  } else if (fabs(data1.y) < fabs(data2.y)) {
    result = data1.y;
  } else if (fabs(data1.y) > fabs(data2.y)) {
    result = data2.y;
  } else {
    result = NAN; // Or any other value to indicate unreliable data
  }
  return result;
}

double _1028A::GPSRedundantSensor::getTheta() {
  pros::c::gps_status_s_t data1, data2;
  data1 = sensor1->get_status();
  data2 = sensor2->get_status();
  double result;
  double difference = fabs(data1.yaw - data2.yaw);

  if (difference <= maxDifference) {
    result = (data1.yaw + data2.yaw) / 2.0;
  } else if (fabs(data1.yaw) < fabs(data2.yaw)) {
    result = data1.yaw;
  } else if (fabs(data1.yaw) > fabs(data2.yaw)) {
    result = data2.yaw;
  } else {
    result = NAN; // Or any other value to indicate unreliable data
  }
  return result;
}