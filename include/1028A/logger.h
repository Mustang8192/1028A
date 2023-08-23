#include "main.h"

namespace _1028A::logger {
namespace file {
void openLogFile(std::string filename);
void logString(std::string str);
void closeLogFile();
void startFileLog();
} // namespace file
static bool Debug = false;
static bool Verbose = false;

enum class Level { DEBUG, INFO, WARN, ERROR, FATAL };

static Level lowestLevel = Level::INFO;

void init();
/**
 * @brief Whether or not to log debug messages.
 *
 * @return true if debug is enabled
 */
bool isDebug();
/**
 * @brief Sets lemlib::debug
 *
 * @param debug the new value
 */
void setDebug(bool debug);

/**
 * @brief Whether or not to log info messages.
 *
 * If false, only log messages with a level of lemlib::logger::Level::WARN
 * or higher will be logged
 */
bool isVerbose();
/**
 * @brief Sets lemlib::verbose
 *
 * @param verbose the new value
 */
void setVerbose(bool verbose);

/**
 * @brief The current lowest log level.
 *
 * @return the lowest loggable level
 */
Level getLowestLevel();

/**
 * @brief Sets the lowest loggable level
 *
 * @param level the new lowest loggable level
 */
void setLowestLevel(Level level);

/**
 * @brief Logs a message with an exception
 *
 * @param level the level of the message
 * @param message the message
 * @param exception the exception
 */
void log(Level level, const char *message, const char *exception);
/**
 * @brief Logs a message
 *
 * @param level the level of the message
 * @param message the message
 */
void log(Level level, const char *message);

/**
 * @brief Logs a debug message
 *
 * @param message
 */
void debug(const char *message);
/**
 * @brief Logs an info message
 *
 * @param message
 */
void info(const char *message);
/**
 * @brief Logs a warning message
 *
 * @param message
 */
void warn(const char *message);
/**
 * @brief Logs an error message
 *
 * @param message
 * @param exception
 */
void error(const char *message, const char *exception);
/**
 * @brief Logs an error message
 *
 * @param message
 */
void error(const char *message);
/**
 * @brief Logs a fatal message
 *
 * @param message
 * @param exception
 */
void fatal(const char *message, const char *exception);
/**
 * @brief Logs a fatal message
 *
 * @param message
 */
void fatal(const char *message);
} // namespace _1028A::logger