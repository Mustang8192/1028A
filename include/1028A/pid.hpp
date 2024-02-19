
#pragma once
#include "pros/rtos.hpp"
#include <string>

namespace _1028A {
class FAPID {
public:
  FAPID(float kF, float kA, float kP, float kI, float kD, std::string name);
  void setGains(float kF, float kA, float kP, float kI, float kD);
  void setExit(float largeError, float smallError, int largeTime, int smallTime,
               int maxTime);
  float update(float target, float position, bool log = false);
  void reset();
  bool settled();
  static void init();

private:
  float kF;
  float kA;
  float kP;
  float kI;
  float kD;

  float largeError;
  float smallError;
  int largeTime = 0;
  int smallTime = 0;
  int maxTime = -1; // -1 means no max time set, run forever

  int largeTimeCounter = 0;
  int smallTimeCounter = 0;
  int startTime = 0;

  float prevError = 0;
  float totalError = 0;
  float prevOutput = 0;

  void log();
  std::string name;
  static std::string input;
  static pros::Task *logTask;
  static pros::Mutex logMutex;
};
} // namespace _1028A
