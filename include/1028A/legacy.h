namespace _1028A::legacy{
    extern int math(float Error, float lastError, float Kp, float Ki, float Kd, double maxSpd);
    extern bool exit(float Error, float Threshold, float currTime, float startTime, float timeExit, float powerValue, float lastError);
    extern void turn(double requestedValue, double spd, double thre, double time, double kpOffset, double kdOffset);
    extern void forward(double RequestedValue, double spd, double thre, double time, double kpOffset, double kdOffset);
    extern void forward(double RequestedValue, double angle, double spd, double thre, double time, double kpOffset, double kdOffset);
}