#include "1028A/logger.h"
#include "1028A/robot.h"
#include "1028A/vars.h"
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <time.h>

std::ofstream logfile;

void _1028A::logger::file::openLogFile(std::string filename) {
  logfile.open(filename, std::ios::out | std::ios::app);
}

void _1028A::logger::file::logString(std::string str) {
  logfile << str << std::endl;
}

void _1028A::logger::file::closeLogFile() { logfile.close(); }

void _1028A::logger::init() {
  robotStartTime = std::time(NULL);
  std::string TIME = ctime(&robotStartTime);
  std::string message = "Initialized at " + TIME;
  info(message.c_str());
}

void _1028A::logger::file::startFileLog() {
  fileLog = true;
  std::string TIME = ctime(&robotStartTime);
  std::string message = "Initialized at " + TIME;
  file::openLogFile("/usd/log.txt");
  file::logString(message);
  file::closeLogFile();
}
/**
 * @brief Whether or not to log debug messages.
 *
 * @return true if debug is enabled
 */
bool _1028A::logger::isDebug() { return Debug; }

/**
 * @brief Sets ::utils::debug
 *
 * @param debug the new value
 */
void _1028A::logger::setDebug(bool DEBUG) { Debug = DEBUG; }

/**
 * @brief Whether or not to log info messages.
 *
 * If false, only log messages with a level of
 * _1028A::logger::Level::WARN or higher will be logged
 */
bool _1028A::logger::isVerbose() { return Verbose; }

/**
 * @brief Sets ::utils::verbose
 *
 * @param verbose the new value
 */
void _1028A::logger::setVerbose(bool VERBOSE) { Verbose = VERBOSE; }

/**
 * @brief The current lowest log level.
 *
 * @return the lowest loggable level
 */
_1028A::logger::Level _1028A::logger::getLowestLevel() {
  return _1028A::logger::lowestLevel;
}

/**
 * @brief Sets the lowest loggable level
 *
 * @param level the new lowest loggable level
 */
void _1028A::logger::setLowestLevel(Level level) {
  _1028A::logger::lowestLevel = level;
}

/*
Util functions for logger.
Not meant to be used outside of this file.
*/

int ordinal(_1028A::logger::Level level) { return static_cast<int>(level); }

const std::string RESET_ANSI = "\033[0m";

std::string getColor(_1028A::logger::Level level) {
  switch (level) {
  case _1028A::logger::Level::DEBUG:
    return "\033[0;36m"; // cyan
  case _1028A::logger::Level::INFO:
    return "\033[0;32m"; // green
  case _1028A::logger::Level::WARN:
    return "\033[0;33m"; // yellow
  case _1028A::logger::Level::ERROR:
    return "\033[0;31m"; // red
  case _1028A::logger::Level::FATAL:
    return "\033[0;31;2m";
  default:
    return RESET_ANSI; // reset (white)
  }
}

std::string getFormattedLevel(_1028A::logger::Level level) {
  const char *name = "";

  switch (level) {
  case _1028A::logger::Level::DEBUG:
    name = "DEBUG";
    break;
  case _1028A::logger::Level::INFO:
    name = "INFO";
    break;
  case _1028A::logger::Level::WARN:
    name = "WARN";
    break;
  case _1028A::logger::Level::ERROR:
    name = "ERROR";
    break;
  case _1028A::logger::Level::FATAL:
    name = "FATAL";
    break;
  default:
    name = "UNKNOWN";
    break;
  }

  return getColor(level) + name;
}

bool checkLowestLevel(_1028A::logger::Level level) {
  return ordinal(level) >= ordinal(_1028A::logger::lowestLevel);
}

/*
End of util functions
*/

/**
 * @brief Logs a message with an exception
 *
 * @param level the level of the message
 * @param message the message
 * @param exception the exception
 */
void _1028A::logger::log(Level level, const char *message,
                         const char *exception) {
  if (!checkLowestLevel(level))
    return;
  /*
  if (level == Level::DEBUG && !Debug)
    return;
  if (level == Level::INFO && !Verbose)
    return;
  */
  if (message == nullptr)
    message = "";
  if (exception == nullptr)
    throw std::invalid_argument("exception cannot be null");

  std::string messageString = "[1028A] " + getFormattedLevel(level) + ": " +
                              message + ": " + exception + RESET_ANSI;

  std::cout << messageString << std::endl;
  if (fileLog) {
    file::openLogFile("/usd/log.txt");
    file::logString(messageString);
    file::closeLogFile();
  }
}

/**
 * @brief Logs a message
 *
 * @param level the level of the message
 * @param message the message
 */
void _1028A::logger::log(Level level, const char *message) {
  if (!checkLowestLevel(level))
    return;
  /*
  if (level == Level::DEBUG && !Debug)
    return;
  if (level == Level::INFO && !Verbose)
    return;
  */
  if (message == nullptr)
    message = "";

  std::string messageString =
      "[1028A] " + getFormattedLevel(level) + ": " + message + RESET_ANSI;

  std::cout << messageString << std::endl;
  if (fileLog) {
    file::openLogFile("/usd/log.txt");
    file::logString(messageString);
    file::closeLogFile();
  }
}

/**
 * @brief Logs a debug message
 *
 * @param message
 */
void _1028A::logger::debug(const char *message) { log(Level::DEBUG, message); }

/**
 * @brief Logs an info message
 *
 * @param message
 */
void _1028A::logger::info(const char *message) { log(Level::INFO, message); }

/**
 * @brief Logs a warning message
 *
 * @param message
 */
void _1028A::logger::warn(const char *message) { log(Level::WARN, message); }

/**
 * @brief Logs an error message
 *
 * @param message
 * @param exception
 */
void _1028A::logger::error(const char *message, const char *exception) {
  log(Level::ERROR, message, exception);
}

/**
 * @brief Logs an error message
 *
 * @param message
 */
void _1028A::logger::error(const char *message) { log(Level::ERROR, message); }

/**
 * @brief Logs a fatal message
 *
 * @param message
 * @param exception
 */
void _1028A::logger::fatal(const char *message, const char *exception) {
  log(Level::FATAL, message, exception);
}

/**
 * @brief Logs a fatal message
 *
 * @param message
 */
void _1028A::logger::fatal(const char *message) { log(Level::FATAL, message); }

void logControllerInputs(const char *filename, uint32_t timestep_ms) {
  FILE *logfile = fopen(
      filename, "w+"); // Open a file for logging (create if it doesn't exist)
  if (logfile == NULL) {
    printf("Failed to open log file!\n");
    return;
  }
  printf("Logging controller inputs to file: %s\n", filename);

  while (1) {
    // Read analog stick values
    int32_t drive_value =
        _1028A::robot::master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int32_t turn_value =
        _1028A::robot::master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    // Read button states
    bool right_button =
        _1028A::robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT);
    bool a_button =
        _1028A::robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_A);
    bool b_button =
        _1028A::robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_B);
    bool x_button =
        _1028A::robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_X);
    bool y_button =
        _1028A::robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_Y);
    bool r1_button =
        _1028A::robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
    bool r2_button =
        _1028A::robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
    bool l1_button =
        _1028A::robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
    bool l2_button =
        _1028A::robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_L2);

    // Write data to the log file on one line
    fprintf(logfile,
            "Drive: %d, Turn: %d, Right: %d, A: %d, B: %d, X: %d, Y: %d, R1: "
            "%d, R2: %d, L1: %d, L2: %d\n",
            drive_value, turn_value, right_button, a_button, b_button, x_button,
            y_button, r1_button, r2_button, l1_button, l2_button);

    fflush(logfile);

    pros::delay(timestep_ms); // Wait for the next time step
  }

  fclose(logfile); // Close the log file when done
}

void simulateActions(const std::string &filename, uint32_t timestep_ms) {
  // Open the input file
  std::ifstream input_file(filename);
  if (!input_file.is_open()) {
    std::cerr << "Failed to open input file: " << filename << std::endl;
    return;
  }

  // Read lines from the input file and simulate actions
  std::string line;
  while (std::getline(input_file, line)) {
    // Parse the line to extract button states and analog values
    int power, turn;
    bool R2, L1, L2, A, Y, B;
    std::istringstream iss(line);
    std::string label;
    iss >> label >> power >> label >> turn >> label >> R2 >> label >> L1 >>
        label >> L2 >> label >> A >> label >> Y >> label >> B;

    // Simulate the actions based on button states and analog values
    std::cout << "Drive: " << power << ", Turn: " << turn << std::endl;

    // Drive control based on analog values
    _1028A::robot::leftMtrs.move(power + turn);
    _1028A::robot::rightMtrs.move(power - turn);

    // Intake simulation
    if (L1) {
      _1028A::robot::intake.move(127); // Spin intake forward
    } else if (L2) {
      _1028A::robot::intake.move(-127); // Spin intake backward
    } else {
      _1028A::robot::intake.move(0); // Stop intake
    }

    // Flap simulation
    if (A) {
      if (Rwing == closed) {
        _1028A::robot::flapR.set_value(1);
        Rwing = open;
        pros::delay(200);
      } else if (Rwing == open) {
        _1028A::robot::flapR.set_value(0);
        Rwing = closed;
        pros::delay(200);
      }
    }
    if (Y) {
      if (Lwing == closed) {
        _1028A::robot::flapL.set_value(1);
        Lwing = open;
        pros::delay(200);
      } else if (Lwing == open) {
        _1028A::robot::flapL.set_value(0);
        Lwing = closed;
        pros::delay(200);
      }
    }
    if (B) {
      if (Lwing != Rwing) {
        _1028A::robot::flapL.set_value(1);
        _1028A::robot::flapR.set_value(1);
        Lwing = open;
        Rwing = open;
        pros::delay(200);
      } else if (Lwing == Rwing && Lwing == closed) {
        _1028A::robot::flapL.set_value(1);
        _1028A::robot::flapR.set_value(1);
        Lwing = open;
        Rwing = open;
        pros::delay(200);
      } else if (Lwing == Rwing && Lwing == open) {
        _1028A::robot::flapL.set_value(0);
        _1028A::robot::flapR.set_value(0);
        Lwing = closed;
        Rwing = closed;
        pros::delay(200);
      }
    }

    // Delay for the specified timestep
    pros::delay(timestep_ms);
  }
}

/*
void simulateActions(const std::string& filename, uint32_t timestep_ms) {
    // Open the input file
    std::ifstream input_file(filename);
    if (!input_file.is_open()) {
        std::cerr << "Failed to open input file: " << filename << std::endl;
        return;
    }

    // loop
    std::string line;
    while (std::getline(input_file, line)) {
        // Parse the line
        int power, turn;
        bool R2, L1, L2, A, Y, B;
        std::istringstream iss(line);
        std::string label;
        iss >> label >> power >> label >> turn >> label >> R2 >> label >> L1 >>
label >> L2 >> label >> A >> label >> Y >> label >> B;

        // Simulate actions
        std::cout << "Drive: " << power << ", Turn: " << turn << std::endl;

        // Drive
        _1028A::robot::leftMtrs.move(power + turn);
        _1028A::robot::rightMtrs.move(power - turn);

        // Intake simulation
        if (L1) {
            _1028A::robot::intake.move(127);
        } else if (L2) {
            _1028A::robot::intake.move(-127);
        } else {
            _1028A::robot::intake.move(0);
        }

        // Flap simulation
        if (A) {
            if (Rwing == closed) {
                _1028A::robot::flapR.set_value(1);
                Rwing = open;
                pros::delay(200);
            } else if (Rwing == open) {
                _1028A::robot::flapR.set_value(0);
                Rwing = closed;
                pros::delay(200);
            }
        }
        if (Y) {
            if (Lwing == closed) {
                _1028A::robot::flapL.set_value(1);
                Lwing = open;
                pros::delay(200);
            } else if (Lwing == open) {
                _1028A::robot::flapL.set_value(0);
                Lwing = closed;
                pros::delay(200);
            }
        }
        if (B) {
            if (Lwing != Rwing) {
                _1028A::robot::flapL.set_value(1);
                _1028A::robot::flapR.set_value(1);
                Lwing = open;
                Rwing = open;
                pros::delay(200);
            } else if (Lwing == Rwing && Lwing == closed) {
                _1028A::robot::flapL.set_value(1);
                _1028A::robot::flapR.set_value(1);
                Lwing = open;
                Rwing = open;
                pros::delay(200);
            } else if (Lwing == Rwing && Lwing == open) {
                _1028A::robot::flapL.set_value(0);
                _1028A::robot::flapR.set_value(0);
                Lwing = closed;
                Rwing = closed;
                pros::delay(200);
            }
        }

        // Delay
        pros::delay(timestep_ms);
    }
}
*/
void clearLogFile(const char *filename) {
  // truncate file
  FILE *logfile = fopen(filename, "w");
  if (logfile == NULL) {
    printf("Failed to open log file!\n");
    return;
  }

  fclose(logfile);
}

/*void opcontrol() {
    logControllerInputs("/usd/logfile.txt", LOG_INTERVAL_MS);
}
*/