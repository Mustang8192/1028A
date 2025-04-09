#include <cmath>
namespace _1028A::legacy{
    extern void forward(double target_distance, double heading, double maxSpd, double timeout, double thre);

    struct PIDGains {
        double kP;
        double kI;
        double kD;
    };

    extern PIDGains autoTuneDrive(double max_distance , int settle_threshold);
}