#include "1028A/logger.h"

void _1028A::logger::init() {
  time_t robotStartTime;
  robotStartTime = std::time(NULL);
  std::string TIME = ctime(&robotStartTime);
  std::string message = "Initialized on " + TIME;
  info(message.c_str());
}
_1028A::logger::Level _1028A::logger::getLowestLevel() {
  return _1028A::logger::lowestLevel;
}


void _1028A::logger::setLowestLevel(Level level) {
  _1028A::logger::lowestLevel = level;
}

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


void _1028A::logger::log(Level level, const char *message,
                         const char *exception) {
  if (!checkLowestLevel(level))
    return;

  if (message == nullptr)
    message = "";
  if (exception == nullptr)
    throw std::invalid_argument("exception cannot be null");

  std::string messageString = "[1028A] " + getFormattedLevel(level) + ": " +
                              message + ": " + exception + RESET_ANSI;

  std::cout << messageString << std::endl;
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
  if (message == nullptr)
    message = "";

  std::string messageString =
      "[1028A] " + getFormattedLevel(level) + ": " + message + RESET_ANSI;

  std::cout << messageString << std::endl;
}

void _1028A::logger::debug(const char *message) { log(Level::DEBUG, message); }

void _1028A::logger::info(const char *message) { log(Level::INFO, message); }

void _1028A::logger::warn(const char *message) { log(Level::WARN, message); }

void _1028A::logger::error(const char *message, const char *exception) {
  log(Level::ERROR, message, exception);
}

void _1028A::logger::error(const char *message) { log(Level::ERROR, message); }

void _1028A::logger::fatal(const char *message, const char *exception) {
  log(Level::FATAL, message, exception);
}

void _1028A::logger::fatal(const char *message) { log(Level::FATAL, message); }