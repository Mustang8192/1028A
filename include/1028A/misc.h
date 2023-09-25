#include "main.h"

namespace _1028A::utils {
/**
* @brief Slew rate limiter
*
* @param target target value
* @param current current value
* @param maxChange maximum change. No maximum if set to 0

* @return float - the limited value
*/
float slew(float target, float current, float maxChange);

/**
 * @brief Convert radians to degrees
 *
 * @param rad radians
 * @return float degrees
 */
float radToDeg(float rad);

/**
 * @brief Convert degrees to radians
 *
 * @param deg degrees
 * @return float radians
 */
float degToRad(float deg);

/**
 * @brief Calculate the error between 2 angles. Useful when calculating the
 * error between 2 headings
 *
 * @param angle1
 * @param angle2
 * @param radians true if angle is in radians, false if not. False by default
 * @return float wrapped angle
 */
float angleError(float angle1, float angle2, bool radians = false);

/**
 * @brief Return the sign of a number
 *
 * @param x the number to get the sign of
 * @return float - -1 if negative, 1 if positive
 */
float sgn(float x);

/**
 * @brief Return the average of a vector of numbers
 *
 * @param values
 * @return float
 */
float avg(std::vector<float> values);

/**
 * @brief Return the average of a vector of numbers
 *
 * @param values
 * @return double
 */
double avg(std::vector<double> values);

double tickToFt(double ticks);

double rpmToFtps(double rpm);

double ftToMeters(double ft);

double metersToFt(double meters);

float reduce_0_to_360(float angle);

float reduce_negative_180_to_180(float angle);

float reduce_negative_90_to_90(float angle);

void init();

void checks();

void ptoSwitch();

} // namespace _1028A::utils