#include "main.h"

namespace _1028A::task {
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
} // namespace _1028A::task