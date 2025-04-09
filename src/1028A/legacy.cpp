#include "1028A/legacy.h"
#include "1028A/robot.h"


void _1028A::legacy::forward(double target_distance_in_inches, double heading = NAN, double maxSpd = 127, double timeout = 2000, double thre = 1) {
  // PID Constants
  const double kP_dist = 0.34;
  const double kI_dist = 0.0;
  const double kD_dist = 1.35;
  const double kP_angle = 1.8;
  const double kI_angle = 0.0;
  const double kD_angle = 0.5;
  const double distance_threshold = thre;
  const double angle_threshold = 2.0;
  const int settle_time = 150;
  const double max_slew = 8.0;

  const double wheel_diameter = 3.25; // in inches
  const double gear_ratio = 36.0 / 48.0; // 0.75
  const double wheel_circumference = M_PI * wheel_diameter;
  const double degrees_per_inch = (300.0 * gear_ratio) / wheel_circumference;

  double target_distance = target_distance_in_inches * degrees_per_inch;

  // Setup
  double initial_heading = robot::inertial.get_heading();
  double target_heading = std::isnan(heading) ? initial_heading : heading;

  double left_start = robot::leftMtrs.get_position();
  double right_start = robot::rightMtrs.get_position();

  double error_dist, last_error_dist = 0, integral_dist = 0;
  double error_angle, last_error_angle = 0, integral_angle = 0;

  double prev_left_power = 0;
  double prev_right_power = 0;

  int start_time = pros::c::millis();
  int within_threshold_start = -1;

  while (true) {
      double left_pos = robot::leftMtrs.get_position() - left_start;
      double right_pos = robot::rightMtrs.get_position() - right_start;
      double avg_pos = (left_pos + right_pos) / 2.0;

      error_dist = target_distance - avg_pos;
      double derivative_dist = error_dist - last_error_dist;

      double dist_output = kP_dist * error_dist + kD_dist * derivative_dist;
      double proposed_integral_dist = integral_dist + error_dist;

      if (std::abs(dist_output) < maxSpd)
          integral_dist = proposed_integral_dist;

      dist_output += kI_dist * integral_dist;
      last_error_dist = error_dist;

      double current_heading = robot::inertial.get_heading();
      error_angle = target_heading - current_heading;

      if (error_angle > 180) error_angle -= 360;
      if (error_angle < -180) error_angle += 360;

      double derivative_angle = error_angle - last_error_angle;
      double angle_output = kP_angle * error_angle + kD_angle * derivative_angle;
      double proposed_integral_angle = integral_angle + error_angle;

      if (std::abs(angle_output) < maxSpd)
          integral_angle = proposed_integral_angle;

      angle_output += kI_angle * integral_angle;
      last_error_angle = error_angle;

      double left_power = dist_output + angle_output;
      double right_power = dist_output - angle_output;

      left_power = std::clamp(left_power, -maxSpd, maxSpd);
      right_power = std::clamp(right_power, -maxSpd, maxSpd);

      left_power = std::clamp(left_power, prev_left_power - max_slew, prev_left_power + max_slew);
      right_power = std::clamp(right_power, prev_right_power - max_slew, prev_right_power + max_slew);

      robot::leftMtrs.move(left_power);
      robot::rightMtrs.move(right_power);

      prev_left_power = left_power;
      prev_right_power = right_power;

      bool within_distance = std::abs(error_dist) < distance_threshold;
      bool within_angle = std::abs(error_angle) < angle_threshold;

      if (within_distance && within_angle) {
          if (within_threshold_start == -1)
              within_threshold_start = pros::c::millis();
          else if (pros::c::millis() - within_threshold_start >= settle_time)
              break;
      } else {
          within_threshold_start = -1;
      }

      if (pros::c::millis() - start_time > timeout)
          break;

      pros::delay(10);
  }

  robot::leftMtrs.move(0);
  robot::rightMtrs.move(0);
}

_1028A::legacy::PIDGains _1028A::legacy::autoTuneDrive(double max_distance = 1000.0, int settle_threshold = 3) {
  double kP = 0.1;
  const double increment = 0.05;

  const double tolerance = 0.001;
  const int max_cycles = 50;
  const int max_runtime = 15000;

  std::vector<double> kP_log;

  for (int cycle = 0; cycle < max_cycles; ++cycle) {
      // Oscillate between 0 and max_distance using only P control
      double left_start = robot::leftMtrs.get_position();
      double right_start = robot::rightMtrs.get_position();
      bool forward = true;
      double last_error = 0;
      int osc_count = 0;
      int zero_crosses = 0;
      std::vector<int> cross_times;

      int start_time = pros::c::millis();

      while (osc_count < 6 && pros::c::millis() - start_time < max_runtime) {
          double left_pos = robot::leftMtrs.get_position() - left_start;
          double right_pos = robot::rightMtrs.get_position() - right_start;
          double avg_pos = (left_pos + right_pos) / 2.0;

          double target = forward ? max_distance : 0.0;
          double error = target - avg_pos;
          double output = std::clamp(kP * error, -100.0, 100.0);

          robot::leftMtrs.move(output);
          robot::rightMtrs.move(output);

          if ((error > 0 && last_error < 0) || (error < 0 && last_error > 0)) {
              cross_times.push_back(pros::c::millis());
              zero_crosses++;
              if (zero_crosses % 2 == 0) osc_count++;
          }

          if (std::abs(error) < 5.0) forward = !forward;

          last_error = error;
          pros::delay(10);
      }

      // Stop motors
      robot::leftMtrs.move(0);
      robot::rightMtrs.move(0);

      if (cross_times.size() < 2) break;

      // Compute Tu
      double Tu = 0;
      for (size_t i = 1; i < cross_times.size(); ++i)
          Tu += (cross_times[i] - cross_times[i - 1]);
      Tu /= (cross_times.size() - 1);
      Tu /= 1000.0;

      double Ku = kP;

      // Ziegler–Nichols PID
      PIDGains tuned;
      tuned.kP = 0.6 * Ku;
      tuned.kI = 1.2 * Ku / Tu;
      tuned.kD = 0.075 * Ku * Tu;

      // Save last kP and check for convergence
      kP_log.push_back(tuned.kP);

      if (kP_log.size() >= settle_threshold) {
          bool settled = true;
          for (size_t i = kP_log.size() - settle_threshold; i < kP_log.size() - 1; ++i) {
              if (std::abs(kP_log[i] - kP_log[i + 1]) > tolerance) {
                  settled = false;
                  break;
              }
          }
          if (settled) return tuned;
      }

      // Increase gain slightly for next cycle
      kP += increment;
  }
  return {0.0, 0.0, 0.0};
}

