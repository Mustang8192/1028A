#include "1028A/logger.h"
#include "1028A/vars.h"
#include <fstream>
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
