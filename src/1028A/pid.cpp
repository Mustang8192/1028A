

#include "1028A/pid.hpp"
#include "1028A/logger.h"
#include "1028A/util.hpp"
#include <iostream>
#include <math.h>

// define static variables
std::string _1028A::FAPID::input = "FAPID";
pros::Task *_1028A::FAPID::logTask = nullptr;
pros::Mutex _1028A::FAPID::logMutex = pros::Mutex();

_1028A::FAPID::FAPID(float kF, float kA, float kP, float kI, float kD,
                     std::string name) {
  this->kF = kF;
  this->kA = kA;
  this->kP = kP;
  this->kI = kI;
  this->kD = kD;
  this->name = name;
}

void _1028A::FAPID::setGains(float kF, float kA, float kP, float kI, float kD) {
  this->kF = kF;
  this->kA = kA;
  this->kP = kP;
  this->kI = kI;
  this->kD = kD;
}

void _1028A::FAPID::setExit(float largeError, float smallError, int largeTime,
                            int smallTime, int maxTime) {
  this->largeError = largeError;
  this->smallError = smallError;
  this->largeTime = largeTime;
  this->smallTime = smallTime;
  this->maxTime = maxTime;
}

float _1028A::FAPID::update(float target, float position, bool log) {
  // check most recent input if logging is enabled
  // this does not run by default because the mutexes could slow down the
  // program calculate output
  float error = target - position;
  float deltaError = error - prevError;
  float output = kF * target + kP * error + kI * totalError + kD * deltaError;
  if (kA != 0)
    output = _1028A::slew(output, prevOutput, kA);
  prevOutput = output;
  prevError = error;
  totalError += error;
  return output;
}

void _1028A::FAPID::reset() {
  prevError = 0;
  totalError = 0;
  prevOutput = 0;
}

bool _1028A::FAPID::settled() {
  if (startTime == 0) { // if maxTime has not been set
    startTime = pros::c::millis();
    return false;
  } else { // check if the FAPID has settled
    if (pros::c::millis() - startTime > maxTime)
      return true;                           // maxTime has been exceeded
    if (std::fabs(prevError) < largeError) { // largeError within range
      if (!largeTimeCounter)
        largeTimeCounter =
            pros::c::millis(); // largeTimeCounter has not been set
      else if (pros::c::millis() - largeTimeCounter > largeTime)
        return true; // largeTime has been exceeded
    }
    if (std::fabs(prevError) < smallError) { // smallError within range
      if (!smallTimeCounter)
        smallTimeCounter =
            pros::c::millis(); // smallTimeCounter has not been set
      else if (pros::c::millis() - smallTimeCounter > smallTime)
        return true; // smallTime has been exceeded
    }
    // if none of the exit conditions have been met
    return false;
  }
}

void _1028A::FAPID::init() {
  if (logTask != nullptr) {
    logTask = new pros::Task{[=] {
      while (true) {
        // get input
        std::cin >> input;
        pros::delay(20);
      }
    }};
  }
}

void _1028A::FAPID::log() {
  // check if the input starts with the name of the FAPID
  // try to obtain the logging mutex
  if (logMutex.take(5)) {
    if (input.find(name) == 0) {
      // remove the name from the input
      input.erase(0, name.length() + 1);
      // check if the input is a function
      if (input == "reset()") {
        reset();
      } else if (input == "kF") {
        std::cout << kF << std::endl;
      } else if (input == "kA") {
        std::cout << kA << std::endl;
      } else if (input == "kP") {
        std::cout << kP << std::endl;
      } else if (input == "kI") {
        std::cout << kI << std::endl;
      } else if (input == "kD") {
        std::cout << kD << std::endl;
      } else if (input == "totalError") {
        std::cout << totalError << std::endl;
      } else if (input.find("kF_") == 0) {
        input.erase(0, 3);
        kF = std::stof(input);
      } else if (input.find("kA_") == 0) {
        input.erase(0, 3);
        kA = std::stof(input);
      } else if (input.find("kP_") == 0) {
        input.erase(0, 3);
        kP = std::stof(input);
      } else if (input.find("kI_") == 0) {
        input.erase(0, 3);
        kI = std::stof(input);
      } else if (input.find("kD_") == 0) {
        input.erase(0, 3);
        kD = std::stof(input);
      }
      // clear the input
      input = "";
    }
    // release the logging mutex
    logMutex.give();
  }
}
