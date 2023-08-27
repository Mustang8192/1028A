#include "main.h"

namespace _1028A {
class GPSRedundantSensor {
private:
  pros::GPS *sensor1;
  pros::GPS *sensor2;
  double maxDifference;

public:
  GPSRedundantSensor(pros::GPS *s1, pros::GPS *s2, double maxDiff);
  double getXCoordinate();
  double getYCoordinate();
  double getTheta();
};
} // namespace _1028A