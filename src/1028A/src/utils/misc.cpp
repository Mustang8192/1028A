#include "../src/1028A/include/api.h"

/*omit
 * @brief Slew rate limiter
 *
 * @param target target value
 * @param current current value
 * @param maxChange maximum change. No maximum if set to 0
 * @return float - the limited value
 */
float _1028A::utils::misc::slew(float target, float current, float maxChange) {
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
float _1028A::utils::misc::radToDeg(float rad) { return rad * 180 / M_PI; }

/*omit
 * @brief Convert degrees to radians
 *
 * @param deg degrees
 * @return float radians
 */
float _1028A::utils::misc::degToRad(float deg) { return deg * M_PI / 180; }

/*omit
 * @brief Calculate the error between 2 angles. Useful when calculating the
 * error between 2 headings
 *
 * @param angle1
 * @param angle2
 * @param radians true if angle is in radians, false if not. False by default
 * @return float wrapped angle
 */
float _1028A::utils::misc::angleError(float angle1, float angle2,
                                      bool radians) {
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
float _1028A::utils::misc::sgn(float x) {
  if (x < 0)
    return -1;
  else
    return 1;
}

/*omit
 * @brief Return the average of a vector of numbers
 *
 * @param values
 * @return float
 */
float _1028A::utils::misc::avg(std::vector<float> values) {
  float sum = 0;
  for (float value : values) {
    sum += value;
  }
  return sum / values.size();
}

/*omit
 * @brief Return the average of a vector of numbers
 *
 * @param values
 * @return double
 */
double _1028A::utils::misc::avg(std::vector<double> values) {
  double sum = 0;
  for (double value : values) {
    sum += value;
  }
  return sum / values.size();
}
