#include "main.h"

extern int math(float Error, float lastError, float Kp, float Ki, float Kd,
                double maxSpd);

extern void turn(double RequestedValue, double spd, double thre, double time,
                 double kpOffset, double kdOffset);

extern void forward(double RequestedValue, double spd, double thre, double time,
                    double kpOffset, double kdOffset);
