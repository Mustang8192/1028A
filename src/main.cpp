#include "main.h"
#include "1028A/api.h"

void initialize() {
  _1028A::utils::init();
  _1028A::robot::chassis.calibrate();
  _1028A::robot::chassis.setPose({0, 0, 0});
}

void disabled() { _1028A::logger::info("Robot Disabled"); }

void competition_initialize() {}

void autonomous() { _1028A::comp::auton::auton(); }

class GPSRedundantSensor {
private:
  pros::GPS *sensor1;   // Pointer to the first GPS sensor
  pros::GPS *sensor2;   // Pointer to the second GPS sensor
  double maxDifference; // Maximum allowable difference between sensor readings

public:
  GPSRedundantSensor(pros::GPS *s1, pros::GPS *s2, double maxDiff)
      : sensor1(s1), sensor2(s2), maxDifference(maxDiff) {}

  void getXCoordinate(double &result) {
    pros::c::gps_status_s_t data1, data2;
    data1 = sensor1->get_status();
    data2 = sensor2->get_status();

    double difference = fabs(data1.x - data2.x);

    if (difference <= maxDifference) {
      result = (data1.x + data2.x) / 2.0;
    } else if (fabs(data1.x) < fabs(data2.x)) {
      result = data1.x;
    } else if (fabs(data1.x) > fabs(data2.x)) {
      result = data2.x;
    } else {
      // No reliable data available
      result = NAN; // Or any other value to indicate unreliable data
    }
  }

  void getYCoordinate(double &result) {
    pros::c::gps_status_s_t data1, data2;
    data1 = sensor1->get_status();
    data2 = sensor2->get_status();

    double difference = fabs(data1.y - data2.y);

    if (difference <= maxDifference) {
      result = (data1.y + data2.y) / 2.0;
    } else if (fabs(data1.y) < fabs(data2.y)) {
      result = data1.y;
    } else if (fabs(data1.y) > fabs(data2.y)) {
      result = data2.y;
    } else {
      // No reliable data available
      result = NAN; // Or any other value to indicate unreliable data
    }
  }

  void getTheta(double &result) {
    pros::c::gps_status_s_t data1, data2;
    data1 = sensor1->get_status();
    data2 = sensor2->get_status();

    double difference = fabs(data1.yaw - data2.yaw);

    if (difference <= maxDifference) {
      result = (data1.yaw + data2.yaw) / 2.0;
    } else if (fabs(data1.yaw) < fabs(data2.yaw)) {
      result = data1.yaw;
    } else if (fabs(data1.yaw) > fabs(data2.yaw)) {
      result = data2.yaw;
    } else {
      // No reliable data available
      result = NAN; // Or any other value to indicate unreliable data
    }
  }
};
void opcontrol() {
  _1028A::logger::info("Driver Control Enabled");
  _1028A::task::Async driveCTRL(_1028A::comp::driver::driveCTRL);
  _1028A::task::Async intakeCTRL(_1028A::comp::driver::intakeCTRL);
  _1028A::task::Async ptoCTRL(_1028A::comp::driver::ptoCTRL);
  while (1) {
    pros::delay(20);
  }
}
