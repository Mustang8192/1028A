#include "1028A/legacy.h"
#include "1028A/robot.h"

int _1028A::legacy::math(float Error, float lastError, float Kp, float Ki, float Kd, double maxSpd){
    float P;
    float D;
    float Drive;

    P=(Kp*Error);
    static float I = 0;
    I += (Ki*Error);
    if(I>1){
        I=1;
    }
    else if(I<-1){
        I=-1;
    }

    D=(Kd*(Error-lastError));
    Drive=P+I+D;
    if(Drive>maxSpd){
        Drive=maxSpd;
    }
    else if(Drive<-maxSpd){
        Drive=-maxSpd;
    }
    return Drive;
}

bool _1028A::legacy::exit(float Error, float Threshold, float currTime, float startTime, float timeExit, float powerValue, float lastError){
    if(Error<Threshold && Error>-Threshold){
        if(currTime-startTime>timeExit){
            return true;
        }
    }
    else{
        startTime=currTime;
        lastError=Error;
    }
    return false;
}