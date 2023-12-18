#include "main.h"

namespace _1028A::legacy {
extern int math(float Error, float lastError, float Kp, float Ki, float Kd,
                double maxSpd);
extern bool exit(float Error, float Threshold, float currTime, float startTime,
                 float timeExit, float powerValue, float lastError);
extern void turn(double RequestedValue, double spd, double thre, double time,
                 double kpOffset, double kdOffset);
extern void ptturn(double RequestedValue, double spd, double thre, double time,
                   double kpOffset, double kdOffset, bool leftlock,
                   bool rightlock);
extern void forward(double RequestedValue, double spd, double thre, double time,
                    double kpOffset, double kdOffset);
extern void forward(double RequestedValue, double spd, double thre, double time,
                    double kpOffset, double kdOffset, bool brake);
extern void forward(double spd, double time);
} // namespace _1028A::legacy