
#include "1028A/misc/task.h"
#include "1028A/misc/json.h"

void _1028A::task::TaskWrapper::loop() {
  // utils::logger::warn("TaskWrapper::loop: loop is not overridden");
}

void _1028A::task::TaskWrapper::startTask(const std::string &iname) {
  if (task) {
    json::string message =
        "TaskWrapper::startTask: task already running: " + task->getName();
    // utils::logger::warn(message.c_str());
  }
  task = std::make_unique<CrossplatformThread>(trampoline, this, iname.c_str());
}

void _1028A::task::TaskWrapper::stopTask() { task = nullptr; }

std::string _1028A::task::TaskWrapper::getName() { return task->getName(); };

void _1028A::task::TaskWrapper::trampoline(void *iparam) {
  static_cast<TaskWrapper *>(iparam)->loop();
}

_1028A::task::Trigger::Function
_1028A::task::Trigger::Function::operator!() && {
  return std::not_fn(std::move(*this));
}

_1028A::task::Trigger &&
_1028A::task::Trigger::requirement(Trigger::Function &&function) {
  requirements.emplace_back(std::move(function));
  return std::move(*this);
}

_1028A::task::Trigger &&
_1028A::task::Trigger::require(Trigger::Function &&function) {
  return requirement(std::move(function));
}

_1028A::task::Trigger &&
_1028A::task::Trigger::exception(Trigger::Function &&function) {
  exceptions.emplace_back(std::move(function));
  return std::move(*this);
}

_1028A::task::Trigger &&
_1028A::task::Trigger::unless(Trigger::Function &&function) {
  return exception(std::move(function));
}

bool _1028A::task::Trigger::run() {
  if (std::any_of(exceptions.begin(), exceptions.end(),
                  [](const auto &function) { return function(); })) {
    return true;
  }
  if (std::all_of(requirements.begin(), requirements.end(),
                  [](const auto &function) { return function(); })) {
    return true;
  }
  return false;
}

bool _1028A::task::Trigger::operator()() { return run(); }

_1028A::task::Async::Async(std::function<void()> &&iaction)
    : action(std::move(iaction)) {
  startTask("Async");
}

_1028A::task::Async::Async(Trigger &&itrigger, std::function<void()> &&iaction)
    : trigger(std::move(itrigger)), action(std::move(iaction)) {
  startTask("Async");
}

void _1028A::task::Async::forceStart() { _forceStart = true; }

void _1028A::task::Async::forceStop() {
  stopTask();
  _complete = true;
}

bool _1028A::task::Async::hasStarted() const { return _started; }

bool _1028A::task::Async::isComplete() const { return _complete; }

void _1028A::task::Async::waitUntilComplete() const {
  while (!_complete) {
    pros::delay(10);
  }
}

void _1028A::task::Async::loop() {
  while (!(trigger() || _forceStart)) {
    pros::delay(10);
  }
  _started = true;
  action();
  _complete = true;
}