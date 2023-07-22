#include "../src/1028A/src/ui/addons/gif/gifclass.hpp"
#include "../src/1028A/src/ui/addons/graphing/grapher.hpp"
#include "../src/1028A/src/utils/addons/grafana/guimanager.h"
#include "../src/1028A/src/utils/addons/grafana/variable.h"
#include "../src/1028A/src/utils/addons/grafana/variablegroup.h"
#include "main.h"
#pragma once

namespace _1028A {

namespace competition {
namespace auton {
void auton();
void redLeft();
void redRight();
void blueLeft();
void blueRight();
void skills();
} // namespace auton

namespace driver {
void driveCTRL();
void auxCTRL();
} // namespace driver
} // namespace competition

class TrackingWheel {
public:
  /**
   * @brief Create a new tracking wheel
   *
   * @param encoder the optical shaft encoder to use
   * @param diameter diameter of the tracking wheel in inches
   * @param distance distance between the tracking wheel and the center of
   * rotation in inches
   * @param gearRatio gear ratio of the tracking wheel, defaults to 1
   */
  TrackingWheel(pros::ADIEncoder *encoder, float diameter, float distance,
                float gearRatio = 1);
  /**
   * @brief Create a new tracking wheel
   *
   * @param encoder the v5 rotation sensor to use
   * @param diameter diameter of the tracking wheel in inches
   * @param distance distance between the tracking wheel and the center of
   * rotation in inches
   * @param gearRatio gear ratio of the tracking wheel, defaults to 1
   */
  TrackingWheel(pros::Rotation *encoder, float diameter, float distance,
                float gearRatio = 1);
  /**
   * @brief Create a new tracking wheel
   *
   * @param motors the motor group to use
   * @param diameter diameter of the drivetrain wheels in inches
   * @param distance half the track width of the drivetrain in inches
   * @param rpm theoretical maximum rpm of the drivetrain wheels
   */
  TrackingWheel(pros::Motor_Group *motors, float diameter, float distance,
                float rpm);
  /**
   * @brief Reset the tracking wheel position to 0
   *
   */
  void reset();
  /**
   * @brief Get the distance traveled by the tracking wheel
   *
   * @return float distance traveled in inches
   */
  float getDistanceTraveled();
  /**
   * @brief Get the offset of the tracking wheel from the center of rotation
   *
   * @return float offset in inches
   */
  float getOffset();
  /**
   * @brief Get the type of tracking wheel
   *
   * @return int - 1 if motor group, 0 otherwise
   */
  int getType();

private:
  float diameter;
  float distance;
  float rpm;
  pros::ADIEncoder *encoder = nullptr;
  pros::Rotation *rotation = nullptr;
  pros::Motor_Group *motors = nullptr;
  float gearRatio = 1;
};

/**
 * @brief Struct containing all the sensors used for odometry
 *
 * The sensors are stored in a struct so that they can be easily passed to the
 * chassis class The variables are pointers so that they can be set to nullptr
 * if they are not used Otherwise the chassis class would have to have a
 * constructor for each possible combination of sensors
 *
 * @param vertical1 pointer to the first vertical tracking wheel
 * @param vertical2 pointer to the second vertical tracking wheel
 * @param horizontal1 pointer to the first horizontal tracking wheel
 * @param horizontal2 pointer to the second horizontal tracking wheel
 * @param imu pointer to the IMU
 */
typedef struct {
  TrackingWheel *vertical1;
  TrackingWheel *vertical2;
  TrackingWheel *horizontal1;
  TrackingWheel *horizontal2;
  pros::Imu *imu;
} OdomSensors_t;

/**
 * @brief Struct containing constants for a chassis controller
 *
 * The constants are stored in a struct so that they can be easily passed to the
 * chassis class Set a constant to 0 and it will be ignored
 *
 * @param kP proportional constant for the chassis controller
 * @param kD derivative constant for the chassis controller
 * @param smallError the error at which the chassis controller will switch to a
 * slower control loop
 * @param smallErrorTimeout the time the chassis controller will wait before
 * switching to a slower control loop
 * @param largeError the error at which the chassis controller will switch to a
 * faster control loop
 * @param largeErrorTimeout the time the chassis controller will wait before
 * switching to a faster control loop
 * @param slew the maximum acceleration of the chassis controller
 */
typedef struct {
  float kP;
  float kD;
  float smallError;
  float smallErrorTimeout;
  float largeError;
  float largeErrorTimeout;
  float slew;
} ChassisController_t;

/**
 * @brief Struct containing constants for a drivetrain
 *
 * The constants are stored in a struct so that they can be easily passed to the
 * chassis class Set a constant to 0 and it will be ignored
 *
 * @param leftMotors pointer to the left motors
 * @param rightMotors pointer to the right motors
 * @param trackWidth the track width of the robot
 * @param wheelDiameter the diameter of the wheels (2.75, 3.25, 4, 4.125)
 * @param rpm the rpm of the wheels
 */
typedef struct {
  pros::Motor_Group *leftMotors;
  pros::Motor_Group *rightMotors;
  float trackWidth;
  float wheelDiameter;
  float rpm;
} Drivetrain_t;

namespace odom {
class Pose {
public:
  /** @brief x value*/
  float x;
  /** @brief y value*/
  float y;
  /** @brief theta value*/
  float theta;
  /**
   * @brief Create a new pose
   *
   * @param x component
   * @param y component
   * @param theta heading. Defaults to 0
   */
  Pose(float x, float y, float theta = 0);
  /**
   * @brief Add a pose to this pose
   *
   * @param other other pose
   * @return Pose
   */
  Pose operator+(const Pose &other);
  /**
   * @brief Subtract a pose from this pose
   *
   * @param other other pose
   * @return Pose
   */
  Pose operator-(const Pose &other);
  /**
   * @brief Multiply a pose by this pose
   *
   * @param other other pose
   * @return Pose
   */
  float operator*(const Pose &other);
  /**
   * @brief Multiply a pose by a float
   *
   * @param other float
   * @return Pose
   */
  Pose operator*(const float &other);
  /**
   * @brief Divide a pose by a float
   *
   * @param other float
   * @return Pose
   */
  Pose operator/(const float &other);
  /**
   * @brief Linearly interpolate between two poses
   *
   * @param other the other pose
   * @param t t value
   * @return Pose
   */
  Pose lerp(Pose other, float t);
  /**
   * @brief Get the distance between two poses
   *
   * @param other the other pose
   * @return float
   */
  float distance(Pose other);
  /**
   * @brief Get the angle between two poses
   *
   * @param other the other pose
   * @return float in radians
   */
  float angle(Pose other);
  /**
   * @brief Rotate a pose by an angle
   *
   * @param angle angle in radians
   * @return Pose
   */
  Pose rotate(float angle);
};
/**
 * @brief Set the sensors to be used for odometry
 *
 * @param sensors the sensors to be used
 * @param drivetrain drivetrain to be used
 */
void setSensors(_1028A::OdomSensors_t sensors, _1028A::Drivetrain_t drivetrain);
/**
 * @brief Get the pose of the robot
 *
 * @param radians true for theta in radians, false for degrees. False by default
 * @return Pose
 */
Pose getPose(bool radians = false);
/**
 * @brief Set the Pose of the robot
 *
 * @param pose the new pose
 * @param radians true if theta is in radians, false if in degrees. False by
 * default
 */
void setPose(Pose pose, bool radians = false);
/**
 * @brief Update the pose of the robot
 *
 */
void update();
/**
 * @brief Initialize the odometry system
 *
 */
void init();
} // namespace odom

namespace ui {} // namespace ui

namespace utils {
namespace misc {
/**
 * @brief Slew rate limiter
 *
 * @param target target value
 * @param current current value
 * @param maxChange maximum change. No maximum if set to 0

 * @return float - the limited value
 */
float slew(float target, float current, float maxChange);

/**
 * @brief Convert radians to degrees
 *
 * @param rad radians
 * @return float degrees
 */
float radToDeg(float rad);

/**
 * @brief Convert degrees to radians
 *
 * @param deg degrees
 * @return float radians
 */
float degToRad(float deg);

/**
 * @brief Calculate the error between 2 angles. Useful when calculating the
 * error between 2 headings
 *
 * @param angle1
 * @param angle2
 * @param radians true if angle is in radians, false if not. False by default
 * @return float wrapped angle
 */
float angleError(float angle1, float angle2, bool radians = false);

/**
 * @brief Return the sign of a number
 *
 * @param x the number to get the sign of
 * @return float - -1 if negative, 1 if positive
 */
float sgn(float x);

/**
 * @brief Return the average of a vector of numbers
 *
 * @param values
 * @return float
 */
float avg(std::vector<float> values);

/**
 * @brief Return the average of a vector of numbers
 *
 * @param values
 * @return double
 */
double avg(std::vector<double> values);
} // namespace misc
namespace checks {
void checkMotor();
void checkBattery();
void checkPorts();
void check();
} // namespace checks
namespace logger {
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
 * @brief Sets debug
 *
 * @param debug the new value
 */
void setDebug(bool debug);

/**
 * @brief Whether or not to log info messages.
 *
 * If false, only log messages with a level of logger::Level::WARN
 * or higher will be logged
 */
bool isVerbose();
/**
 * @brief Sets verbose
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
} // namespace logger
namespace pid {
/**
 * @brief Feedforward, Acceleration, Proportional, Integral, Derivative PID
 * controller
 *
 * The controller does not loop on its own. It must be called in a loop.
 * For example: while(!controller.settled) { controller.update(input, output); }
 *
 */
class FAPID {
public:
  /**
   * @brief Construct a new FAPID
   *
   * @param kF feedfoward gain, multiplied by target and added to output. Set 0
   * if disabled
   * @param kA acceleration gain, limits the change in output. Set 0 if disabled
   * @param kP proportional gain, multiplied by error and added to output
   * @param kI integral gain, multiplied by total error and added to output
   * @param kD derivative gain, multiplied by change in error and added to
   * output
   * @param name name of the FAPID. Used for logging
   */
  FAPID(float kF, float kA, float kP, float kI, float kD, std::string name);
  /**
   * @brief Set gains
   *
   * @param kF feedfoward gain, multiplied by target and added to output. Set 0
   * if disabled
   * @param kA acceleration gain, limits the change in output. Set 0 if disabled
   * @param kP proportional gain, multiplied by error and added to output
   * @param kI integral gain, multiplied by total error and added to output
   * @param kD derivative gain, multiplied by change in error and added to
   * output
   */
  void setGains(float kF, float kA, float kP, float kI, float kD);
  /**
   * @brief Set the exit conditions
   *
   * @param largeError range where error is considered large
   * @param smallError range where error is considered small
   * @param largeTime time in milliseconds t
   * @param smallTime
   * @param maxTime
   */
  void setExit(float largeError, float smallError, int largeTime, int smallTime,
               int maxTime);
  /**
   * @brief Update the FAPID
   *
   * @param target the target value
   * @param position the current value
   * @param log whether to check the most recent terminal input for user input.
   * Default is false because logging multiple PIDs could slow down the program.
   * @return float - output
   */
  float update(float target, float position, bool log = false);
  /**
   * @brief Reset the FAPID
   */
  void reset();
  /**
   * @brief Check if the FAPID has settled
   *
   * If the exit conditions have not been set, this function will always return
   * false
   *
   * @return true - the FAPID has settled
   * @return false - the FAPID has not settled
   */
  bool settled();
  /**
   * @brief initialize the FAPID logging system
   *
   * if this function is called, std::cin will be used to interact with the
   * FAPID
   *
   * the user can interact with the FAPID through the terminal
   * the user can access gains and other variables with the following format:
   * <name>.<variable> to get the value of the variable
   * <name>.<variable>_<value> to set the value of the variable
   * for example:
   * pid.kP_0.5 will set the kP value to 0.5
   * list of variables thats value can be set:
   * kF, kA, kP, kI, kD
   * list of variables that can be accessed:
   * kF, kA, kP, kI, kD, totalError
   * list of functions that can be called:
   * reset()
   */
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
} // namespace pid
namespace task {
/**
 * A utility class that wraps a task trampoline. To use, inherit your class from
 * TaskWrapper and override the `loop` method. To start the task, the
 * `startTask` method must be called, either from the derived constructor or
 * from outside the class.
 */
class TaskWrapper {
public:
  TaskWrapper(const TaskWrapper &) = delete;
  TaskWrapper operator=(const TaskWrapper &) = delete;
  TaskWrapper(TaskWrapper &&) = default;
  TaskWrapper &operator=(TaskWrapper &&) = default;
  virtual ~TaskWrapper() = default;

protected:
  TaskWrapper() = default;

  /**
   * Override this function to implement a custom task loop.
   * Will throw if not overridden.
   */
  virtual void loop();

public:
  /**
   * Start the task.
   *
   * @param iname The task name, optional.
   */
  virtual void startTask(const std::string &iname = "TaskWrapper");

  /**
   * Kill the task.
   */
  virtual void stopTask();

  /**
   * Get the task name.
   *
   * @return The name.
   */
  virtual std::string getName();

private:
  static void trampoline(void *iparam);
  std::unique_ptr<CrossplatformThread> task{nullptr};
};
/**
 * A Trigger is a collection of functions. When all requirements are met, or any
 * exceptions are met, then the Trigger will return true. Used for determining
 * when to settle for PID movements and triggering async actions.
 */
class Trigger {
public:
  /**
   * A function wrapper that supports the not operator.
   */
  class Function : public std::function<bool()> {
  public:
    using function::function;
    Function operator!() &&;
  };

  /**
   * Trigger constructors.
   */
  Trigger() = default;
  Trigger(const Trigger &) = delete;
  Trigger operator=(const Trigger &) = delete;
  Trigger(Trigger &&) = default;
  Trigger &operator=(Trigger &&) = default;
  virtual ~Trigger() = default;

  /**
   * Add a requirement. The trigger will only fire if all requirements are met.
   *
   * @param function The requirement
   */
  virtual Trigger &&requirement(Function &&function);
  virtual Trigger &&require(Function &&function);

  /**
   * Add an exception. The trigger will fire if any of the exceptions are met.
   *
   * @param function The exception
   */
  virtual Trigger &&exception(Function &&function);
  virtual Trigger &&unless(Function &&function);

  /**
   * Run all the requirements and exceptions.
   *
   * @return Whether the trigger has been fired
   */
  virtual bool run();
  virtual bool operator()();

  /**
   * Trigger if the time since the first call of the function is greater than a
   * value.
   *
   * @param  time The time
   * @return A function that triggers when the time since the first call of the
   * function is greater than a value.
   */
  static Function time(const okapi::QTime &time);

protected:
  std::vector<Function> requirements;
  std::vector<Function> exceptions;
};
/**
 * A helper class for running actions asynchronously. You use the class by
 * passing it a function to run and an optional trigger. It will then run the
 * function in a separate task after the trigger has fired.
 */
class Async : public TaskWrapper {
public:
  /**
   * Run a function asynchronously.
   *
   * @param iaction The function to run.
   */
  explicit Async(std::function<void()> &&iaction);

  /**
   * Run a function asynchronously after a trigger fires.
   *
   * @param itrigger The trigger to wait for before running the action.
   * @param iaction  The function to run.
   */
  explicit Async(Trigger &&itrigger, std::function<void()> &&iaction);

  /**
   * If a trigger was specified, force it to fire and run the action.
   */
  virtual void forceStart();

  /**
   * Stop the internal task.
   */
  virtual void forceStop();

  /**
   * Whether the action has been started.
   *
   * @return True if started, False otherwise.
   */
  virtual bool hasStarted() const;

  /**
   * Whether the action has completed and returned.
   *
   * @return True if the action is complete, False otherwise.
   */
  virtual bool isComplete() const;

  /**
   * Wait until the action is complete.
   */
  virtual void waitUntilComplete() const;

protected:
  Trigger trigger{};
  std::function<void()> action{nullptr};

  bool _forceStart{false};
  bool _started{false};
  bool _complete{false};

  void loop() override;
};
namespace addons {}
} // namespace task
} // namespace utils

/**
 * @brief Chassis class
 *
 */
class Chassis {
public:
  /**
   * @brief Construct a new Chassis
   *
   * @param drivetrain drivetrain to be used for the chassis
   * @param lateralSettings settings for the lateral controller
   * @param angularSettings settings for the angular controller
   * @param sensors sensors to be used for odometry
   */
  Chassis(Drivetrain_t drivetrain, ChassisController_t lateralSettings,
          ChassisController_t angularSettings, OdomSensors_t sensors);
  /**
   * @brief Calibrate the chassis sensors
   *
   */
  void calibrate();
  /**
   * @brief Set the pose of the chassis
   *
   * @param x new x value
   * @param y new y value
   * @param theta new theta value
   * @param radians true if theta is in radians, false if not. False by default
   */
  void setPose(double x, double y, double theta, bool radians = false);
  /**
   * @brief Set the pose of the chassis
   *
   * @param pose the new pose
   * @param radians whether pose theta is in radians (true) or not (false).
   * false by default
   */
  void setPose(odom::Pose pose, bool radians = false);
  /**
   * @brief Get the pose of the chassis
   *
   * @param radians whether theta should be in radians (true) or degrees
   * (false). false by default
   * @return Pose
   */
  odom::Pose getPose(bool radians = false);
  /**
   * @brief Turn the chassis so it is facing the target point
   *
   * The PID logging id is "angularPID"
   *
   * @param x x location
   * @param y y location
   * @param timeout longest time the robot can spend moving
   * @param reversed whether the robot should turn in the opposite direction.
   * false by default
   * @param maxSpeed the maximum speed the robot can turn at. Default is 200
   * @param log whether the chassis should log the turnTo function. false by
   * default
   */
  void turnTo(float x, float y, int timeout, bool reversed = false,
              float maxSpeed = 127, bool log = false);
  /**
   * @brief Move the chassis towards the target point
   *
   * The PID logging ids are "angularPID" and "lateralPID"
   *
   * @param x x location
   * @param y y location
   * @param timeout longest time the robot can spend moving
   * @param maxSpeed the maximum speed the robot can move at
   * @param log whether the chassis should log the turnTo function. false by
   * default
   */
  void moveTo(float x, float y, int timeout, float maxSpeed = 200,
              bool log = false);
  /**
   * @brief Move the chassis along a path
   *
   * @param filePath file path to the path. No need to preface it with /usd/
   * @param timeout the maximum time the robot can spend moving
   * @param lookahead the lookahead distance. Units in inches. Larger values
   * will make the robot move faster but will follow the path less accurately
   * @param reverse whether the robot should follow the path in reverse. false
   * by default
   * @param maxSpeed the maximum speed the robot can move at
   * @param log whether the chassis should log the path on a log file. false by
   * default.
   */
  void follow(const char *filePath, int timeout, float lookahead,
              bool reverse = false, float maxSpeed = 127, bool log = false);

private:
  ChassisController_t lateralSettings;
  ChassisController_t angularSettings;
  Drivetrain_t drivetrain;
  OdomSensors_t odomSensors;
};
} // namespace _1028A