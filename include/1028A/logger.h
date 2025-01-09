#include "main.h"

namespace _1028A::logger{
    enum class Level { DATA, DEBUG, INFO, WARN, ERROR, FATAL };
    static Level lowestLevel = Level::INFO;

    void init();
    Level getLowestLevel();
    void setLowestLevel(Level level);
    void log(Level level, const char *message, const char *exception);
    void log(Level level, const char *message);

    void log(Level level, const char *message, std::string filename,
            bool logToFile);
    void debug(const char *message);
    void info(const char *message);
    void warn(const char *message);
    void error(const char *message, const char *exception);
    void error(const char *message);
    void fatal(const char *message, const char *exception);
    void fatal(const char *message);
}