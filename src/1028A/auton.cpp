#include "1028A/auton.h"
#include "1028A/legacy.h"
#include "1028A/logger.h"
#include "1028A/robot.h"
#include "lemlib/chassis/chassis.hpp"
#include "pros/rtos.hpp"
#include <cmath>
#include <queue>

double armtarget = 0;
int reset = 0;
int armOverride = 0;
int hasreset = 0;
int _1028A::auton::autonStop = 0;
int _1028A::auton::autonSelect = 0;

void _1028A::auton::lbTask() {
  double rotationValue = 0;
  double armP = 0.0005;
  double armD = 0.565;
  double armError = 0;
  double armPrevError = 0;
  double threshold = 0.5;
  double armSpeed = 0;
  double rotationRaw = 0;
  double derivative = 0;
  _1028A::robot::LB.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
  _1028A::robot::LBS.set_reversed(1);
  _1028A::robot::LBS.reset_position();

  while (true) {
    rotationRaw = -_1028A::robot::LBS.get_position();
    rotationValue = rotationRaw / 100;

    armError = armtarget - rotationValue;

    armSpeed = (armError * armP) + (armPrevError * armD);

    // lB.move(armSpeed);

    if (fabs(armError) <= threshold) {
      armSpeed = 0;
    }

    // if (_1028A::robot::LBSLimit.get_value() && hasreset == 0){
    //   _1028A::robot::LBS.reset_position();
    //   hasreset = 1;
    // }
    // else{
    //   hasreset = 0;
    // }

    if (reset) {
      _1028A::robot::LB.move(127);
      pros::delay(200);
      _1028A::robot::LB.move(-40);
      pros::delay(200);
      while (1) {
        if (fabs(_1028A::robot::LBS.get_velocity()) < 50) {
          _1028A::robot::LBS.set_position(0);
          armtarget = 0;
          pros::delay(200);
          _1028A::robot::LB.move(0);
          reset = 0;
          break;
        }
        pros::delay(20);
      }
    } else {
      if (!armOverride or !auton::autonStop) {
        _1028A::robot::LB.move(armSpeed);
      }
    }

    if (!pros::competition::is_autonomous() &&
        !pros::competition::is_disabled()) {
      break;
    }

    armPrevError = armError;

    pros::delay(10);
  }
}

enum intakeState { INTAKE, OUTTAKE, STOP, ColorSortRed, ColorSortBlue };
intakeState intake = STOP;
void _1028A::auton::intakeTask() {
  bool kickout = false;
  const int timeThreshold = 1000;
  const int velocityThreshold = 5;
  int count = 0;
  int lastCount = 0;
  while (1) {
    if (!pros::competition::is_autonomous() &&
        !pros::competition::is_disabled() &&
        pros::competition::is_connected()) {
      break;
    }
    if (!autonStop) {
      if (intake == INTAKE) {
        _1028A::robot::optical.set_led_pwm(0);
        _1028A::robot::intake.move(127);
        int initialVelocity = _1028A::robot::intake.get_actual_velocity();

        if (abs(initialVelocity) < velocityThreshold) {
          pros::delay(timeThreshold);
          int currentVelocity = _1028A::robot::intake.get_actual_velocity();

          if (abs(currentVelocity) < velocityThreshold) {
            _1028A::robot::intake.move(-127);
            pros::delay(300);
            _1028A::robot::intake.move(127);
          }
        }
      } else if (intake == OUTTAKE) {
        _1028A::robot::optical.set_led_pwm(0);
        _1028A::robot::intake.move(-127);
      } else if (intake == STOP) {
        _1028A::robot::optical.set_led_pwm(0);
        _1028A::robot::intake.move(0);
      } else if (intake == ColorSortBlue) {

      } else if (intake == ColorSortRed) {
      }
    }
  }
}

int Pause = 0;
void checkIntake() {
  const int timeThreshold = 500;
  const int velocityThreshold = 5;
  while (1) {
    if (!pros::competition::is_autonomous() &&
        !pros::competition::is_disabled() &&
        pros::competition::is_connected()) {
      break;
    }

    if (Pause != 1 or !_1028A::auton::autonStop) {
      int initialVelocity = _1028A::robot::intake.get_actual_velocity();

      if (abs(initialVelocity) < velocityThreshold) {
        pros::delay(timeThreshold);
        int currentVelocity = _1028A::robot::intake.get_actual_velocity();

        if (abs(currentVelocity) < velocityThreshold) {
          _1028A::robot::intake.move(-127);
          pros::delay(300);
          _1028A::robot::intake.move(127);
        }
      }
    }
    pros::delay(20);
  }
}

void odomRead() {
  while (1) {
    if (1) {
      std::string data =
          "(" + std::to_string(_1028A::robot::chassis.getPose().x) + ", " +
          std::to_string(_1028A::robot::chassis.getPose().y) + ", " +
          std::to_string(_1028A::robot::chassis.getPose().theta) + ")";
      _1028A::logger::info(data.c_str());
    }
    pros::delay(400);
  }
}

void Disk() {
  _1028A::robot::intake.move(127);
  while (1) {
    if (_1028A::robot::optical.get_proximity() > 253) {
      _1028A::robot::intake.move(0);
      break;
    }
    pros::delay(20);
  }
}

void queueDisk() {
  _1028A::robot::intake.move(127);
  while (1) {
    if (_1028A::robot::optical.get_proximity() > 253 or
        _1028A::robot::Ldistance.get() < 50) {
      _1028A::robot::intake.move(0);
      break;
    }
    pros::delay(5);
  }
}
enum class DiskColor { NONE, BLUE, RED, UNKNOWN };

struct DiskInfo {
  DiskColor color;
  uint32_t entryTime;
};

enum class IntakeState { INTAKE, EXPEL, COLOR_SORT_BLUE, COLOR_SORT_RED, STOP };

static IntakeState g_currentState = IntakeState::STOP;
static bool g_lastSawDisc = false;
static std::queue<DiskInfo> g_diskQueue;
static bool g_jamTimerActive = false;
static uint32_t g_jamStartTime = 0;
static bool g_seenTopDisc = false;
static uint32_t g_topDiscFirstSeenTime = 0;
constexpr int INTAKE_SPEED = 127;
constexpr int EXPEL_SPEED = 127;
constexpr int REVERSE_SPEED = 127;
constexpr int EJECT_TIME_MS = 300;
constexpr double TOP_DETECTION_THRESH = 50.0;
constexpr int COLOR_SORT_DELAY_MS = 0;
constexpr double JAM_VELOCITY_DIFF = 50.0;
constexpr int JAM_TIME_THRESHOLD_MS = 500;
constexpr int DEJAM_REVERSE_TIME_MS = 300;
constexpr double BLUE_HUE_MIN = 200.0;
constexpr double BLUE_HUE_MAX = 300.0;
constexpr double RED_HUE_MIN = 0.0;
constexpr double RED_HUE_MAX = 40.0;
constexpr uint32_t DISC_TIMEOUT_MS = 900;
constexpr uint32_t SECOND_SENSOR_DELAY_MS = 200;

void setIntakeState(IntakeState newState) { g_currentState = newState; }

static DiskColor getDiskColor() {
  double hue1 = _1028A::robot::optical.get_hue();

  if (hue1 >= BLUE_HUE_MIN && hue1 <= BLUE_HUE_MAX) {
    return DiskColor::BLUE;
    //_1028A::robot::master.rumble(".");
  } else if ((hue1 >= RED_HUE_MIN && hue1 <= RED_HUE_MAX) ||
             (hue1 >= 350.0 && hue1 <= 360.0)) {
    return DiskColor::RED;
  }

  // Fallback to second sensor
  double hue2 = _1028A::robot::opticalH.get_hue();

  if (hue2 >= BLUE_HUE_MIN && hue2 <= BLUE_HUE_MAX) {
    std::cout << "[DEBUG] Second sensor used: Detected BLUE\n";
    return DiskColor::BLUE;
  } else if ((hue2 >= RED_HUE_MIN && hue2 <= RED_HUE_MAX) ||
             (hue2 >= 330.0 && hue2 <= 360.0)) {
    std::cout << "[DEBUG] Second sensor used: Detected RED\n";
    return DiskColor::RED;
  }

  std::cout << "[DEBUG] Second sensor used: Still UNKNOWN\n";
  return DiskColor::UNKNOWN;
}

void tryUpdateUnknownDiskColor() {
  if (g_diskQueue.empty())
    return;

  DiskInfo &frontDisk = g_diskQueue.front();

  if (frontDisk.color == DiskColor::UNKNOWN &&
      pros::millis() - frontDisk.entryTime >= SECOND_SENSOR_DELAY_MS) {
    double hue2 = _1028A::robot::opticalH.get_hue();

    if (hue2 >= BLUE_HUE_MIN && hue2 <= BLUE_HUE_MAX) {
      frontDisk.color = DiskColor::BLUE;
      std::cout << "[DEBUG] Updated UNKNOWN to BLUE via second sensor\n";
    } else if ((hue2 >= RED_HUE_MIN && hue2 <= RED_HUE_MAX) ||
               (hue2 >= 330.0 && hue2 <= 360.0)) {
      frontDisk.color = DiskColor::RED;
      std::cout << "[DEBUG] Updated UNKNOWN to RED via second sensor\n";
    }
  }
}

static const char *colorToString(DiskColor c) {
  switch (c) {
  case DiskColor::BLUE:
    return "BLUE";
  case DiskColor::RED:
    return "RED";
  case DiskColor::UNKNOWN:
    return "UNKNOWN";
  default:
    return "NONE";
  }
}

void intakeControlTask(void *param) {
  _1028A::robot::optical.set_led_pwm(100);
  _1028A::robot::opticalH.set_led_pwm(100);

  while (true) {
    if (!pros::competition::is_autonomous() &&
        !pros::competition::is_disabled()) {
      break;
    }

    bool seesDisc = (_1028A::robot::optical.get_proximity() > 200);
    if (seesDisc && !g_lastSawDisc) {
      DiskColor c = getDiskColor();
      DiskInfo newDisk{c, pros::millis()};
      g_diskQueue.push(newDisk);

      std::cout << "[DEBUG] ** New Disc Detected **  Color=" << colorToString(c)
                << "  entryTime=" << newDisk.entryTime << "\n";
    }
    g_lastSawDisc = seesDisc;

    tryUpdateUnknownDiskColor();

    int targetVelocity = 0;
    int direction = 1;

    switch (g_currentState) {
    case IntakeState::INTAKE:
      targetVelocity = INTAKE_SPEED;
      direction = +1;
      break;
    case IntakeState::EXPEL:
      targetVelocity = EXPEL_SPEED;
      direction = -1;
      break;
    case IntakeState::COLOR_SORT_BLUE:
    case IntakeState::COLOR_SORT_RED:
      targetVelocity = INTAKE_SPEED;
      direction = +1;
      break;
    case IntakeState::STOP:
    default:
      targetVelocity = 0;
      direction = 0;
      break;
    }
    _1028A::robot::intake.move(targetVelocity * direction);

    if (g_currentState == IntakeState::COLOR_SORT_BLUE ||
        g_currentState == IntakeState::COLOR_SORT_RED) {
      double topDist = _1028A::robot::distance.get();
      bool topHasDisc = (topDist < TOP_DETECTION_THRESH);

      if (topHasDisc && !g_seenTopDisc && !g_diskQueue.empty()) {
        g_seenTopDisc = true;
        g_topDiscFirstSeenTime = pros::millis();

        std::cout << "[DEBUG] Top disc first seen at time="
                  << g_topDiscFirstSeenTime << "\n";
      }

      if (!topHasDisc) {
        g_seenTopDisc = false;
      }

      if (g_seenTopDisc &&
          (pros::millis() - g_topDiscFirstSeenTime >= COLOR_SORT_DELAY_MS) &&
          !g_diskQueue.empty()) {
        DiskColor colorToEject =
            (g_currentState == IntakeState::COLOR_SORT_BLUE) ? DiskColor::BLUE
                                                             : DiskColor::RED;

        DiskInfo frontInfo = g_diskQueue.front();
        std::cout << "[DEBUG] Checking front disc: color="
                  << colorToString(frontInfo.color)
                  << "  in queue for=" << (pros::millis() - frontInfo.entryTime)
                  << " ms\n";

        if (frontInfo.color == colorToEject ||
            frontInfo.color == DiskColor::UNKNOWN) {
          std::cout << "[DEBUG] --> Ejecting disc color="
                    << colorToString(frontInfo.color) << "\n";

          pros::delay(80);
          _1028A::robot::intake.move(-EXPEL_SPEED);
          pros::delay(EJECT_TIME_MS);

          _1028A::robot::intake.move(targetVelocity);

          g_diskQueue.pop();
        } else {
          std::cout
              << "[DEBUG] --> Disc color doesn't match target; not ejecting.\n";
        }

        g_seenTopDisc = false;
      }
    }

    while (!g_diskQueue.empty()) {
      DiskInfo &front = g_diskQueue.front();
      if (pros::millis() - front.entryTime > DISC_TIMEOUT_MS) {
        std::cout << "[DEBUG] Removing disc due to time limit. Color="
                  << colorToString(front.color) << "\n";
        g_diskQueue.pop();
      } else {
        break;
      }
    }

    pros::delay(10);
  }
}

void _1028A::auton::auton() {
  autonSelect = 7;
  robot::leftMtrs.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
  robot::rightMtrs.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);

  if (autonSelect == 1) {
    // redSoloSigAWP

    pros::Task checkintake{checkIntake};
    pros::Task lbTask(_1028A::auton::lbTask);
    robot::LBS.set_position(100);
    armtarget = 520;
    robot::chassis.moveToPoint(0, 5, 500, {}, false);
    robot::chassis.moveToPose(-22, -24.5, 60, 3000,
                              {.forwards = false, .minSpeed = 80}, true);
    robot::chassis.waitUntil(48.5);
    robot::mogo.set_value(1);
    robot::chassis.waitUntilDone();
    robot::chassis.moveToPoint(11, -28, 1000, {.earlyExitRange = 2}, false);
    pros::Task intakeTask(intakeControlTask);
    setIntakeState(IntakeState::COLOR_SORT_BLUE);
    // legacy::forward(-12, NAN, 127, 1000, 1);
    robot::chassis.moveToPoint(43, -30, 1900, {.minSpeed = 20}, false);
    robot::leftMtrs.move(127);
    robot::rightMtrs.move(127);
    pros::delay(600);
    robot::leftMtrs.move(0);
    robot::rightMtrs.move(0);
    robot::chassis.turnToPoint(-22, 30, 800, {.earlyExitRange = 2}, false);
    robot::chassis.moveToPoint(-39, 40, 4000,
                               {.maxSpeed = 60, .earlyExitRange = 2}, true);
    reset = 1;
    robot::chassis.waitUntil(40);
    robot::mogo.set_value(0);
    intakeTask.suspend();
    checkintake.suspend();
    robot::intake.move(-127);
    pros::delay(100);
    robot::intake.move(127);
    robot::chassis.waitUntil(82);
    robot::intake.move(0);

    robot::chassis.waitUntilDone();
    robot::chassis.turnToHeading(5, 800, {}, false);
    legacy::forward(-28, NAN, 127, 1000, 1);
    robot::mogo.set_value(1);
    pros::delay(100);
    robot::chassis.turnToPoint(-64, 42, 800, {}, false);
    robot::intake.move(127);
    intakeTask.resume();
    robot::chassis.moveToPoint(-64, 42, 1000, {}, false);
    robot::chassis.turnToPoint(-41, 15, 800, {}, false);
    robot::chassis.moveToPoint(-45, 4, 1000, {.maxSpeed = 80}, false);
  } else if (autonSelect == 2) {
    // blueSoloSigAWP

    pros::Task checkintake{checkIntake};
    pros::Task lbTask(_1028A::auton::lbTask);
    robot::LBS.set_position(100);
    armtarget = 520;
    robot::chassis.moveToPoint(0, 5, 500, {}, false);
    robot::chassis.moveToPose(-22, -24.5, 60, 3000,
                              {.forwards = false, .minSpeed = 80}, true);
    robot::chassis.waitUntil(48.5);
    robot::mogo.set_value(1);
    robot::chassis.waitUntilDone();
    robot::chassis.moveToPoint(11, -28, 1000, {.earlyExitRange = 2}, false);
    pros::Task intakeTask(intakeControlTask);
    setIntakeState(IntakeState::COLOR_SORT_RED);
    // legacy::forward(-12, NAN, 127, 1000, 1);
    robot::chassis.moveToPoint(43, -30, 1900, {.minSpeed = 20}, false);
    robot::leftMtrs.move(127);
    robot::rightMtrs.move(127);
    pros::delay(600);
    robot::leftMtrs.move(0);
    robot::rightMtrs.move(0);
    robot::chassis.turnToPoint(-22, 30, 800, {.earlyExitRange = 2}, false);
    robot::chassis.moveToPoint(-39, 40, 1000,
                               {.maxSpeed = 90, .earlyExitRange = 2}, true);
    reset = 1;
    robot::chassis.waitUntil(40);
    robot::mogo.set_value(0);
    intakeTask.suspend();
    checkintake.suspend();
    robot::intake.move(-127);
    pros::delay(100);
    robot::intake.move(127);
    robot::chassis.waitUntil(82);
    robot::intake.move(0);

    robot::chassis.waitUntilDone();
    robot::chassis.turnToHeading(5, 800, {}, false);
    legacy::forward(-28, NAN, 127, 1000, 1);
    robot::mogo.set_value(1);
    pros::delay(100);
    robot::chassis.turnToPoint(-64, 42, 800, {}, false);
    robot::intake.move(127);
    intakeTask.resume();
    robot::chassis.moveToPoint(-64, 42, 1000, {}, false);
    robot::chassis.turnToPoint(-41, 15, 800, {}, false);
    robot::chassis.moveToPoint(-45, 4, 1000, {.maxSpeed = 80}, false);

  } else if (autonSelect == 4) {
    // Blue 6+1+CORNER
    robot::chassis.setPose(0, -2.5, 0);
    pros::Task lbTask(_1028A::auton::lbTask);
    robot::LBS.set_position(100);
    robot::chassis.swingToHeading(28, lemlib::DriveSide::RIGHT, 400,
                                  {.minSpeed = 60, .earlyExitRange = 2}, false);
    armtarget = 580;
    pros::delay(500);
    robot::chassis.moveToPoint(-11, -19, 2000,
                               {.forwards = false,
                                .maxSpeed = 95,
                                .minSpeed = 40,
                                .earlyExitRange = 3},
                               true);
    robot::chassis.waitUntil(6);
    reset = 1;
    robot::chassis.moveToPoint(-10, -35, 2000,
                               {.forwards = false,
                                .maxSpeed = 95,
                                .minSpeed = 35,
                                .earlyExitRange = 3},
                               true);
    robot::chassis.waitUntil(28);
    robot::mogo.set_value(1);
    robot::chassis.waitUntilDone();
    pros::Task intakeTask(intakeControlTask);
    setIntakeState(IntakeState::COLOR_SORT_RED);
    // pros::Task intakeTask(intakeControlTask);
    // setIntakeState(IntakeState::COLOR_SORT_RED);
    robot::intake.move(127);
    robot::chassis.turnToPoint(-41, -38, 700,
                               {.minSpeed = 50, .earlyExitRange = 2}, false);
    robot::chassis.moveToPoint(
        -46, -36.4, 2000,
        {.maxSpeed = 95, .minSpeed = 55, .earlyExitRange = 11}, false);
    pros::delay(100);
    robot::chassis.swingToPoint(10, -25, lemlib::DriveSide::RIGHT, 1000,
                                {.minSpeed = 80, .earlyExitRange = 4}, false);
    robot::leftMtrs.move(127);
    robot::rightMtrs.move(127);
    pros::delay(50);
    robot::leftMtrs.move(0);
    robot::rightMtrs.move(0);
    robot::chassis.turnToPoint(-45, 4, 1000,
                               {.minSpeed = 0, .earlyExitRange = 4}, false);
    robot::chassis.moveToPoint(-45.5, 8, 1000,
                               {.minSpeed = 50, .earlyExitRange = 2}, false);
    robot::leftMtrs.move(127);
    robot::rightMtrs.move(127);
    pros::delay(400);
    robot::leftMtrs.move(0);
    robot::rightMtrs.move(0);
    // robot::chassis.moveToPoint(-36, 6, 1000, {.forwards = false, .minSpeed =
    // 30}, false);
    pros::delay(100);
    robot::chassis.turnToHeading(100, 1000, {}, false);
    robot::stickL.set_value(1);
    pros::delay(100);
    robot::chassis.moveToPoint(4, 6, 1800,
                               {.minSpeed = 70, .earlyExitRange = 4}, false);
    robot::chassis.moveToPoint(35, 6, 1800,
                               {.maxSpeed = 70, .earlyExitRange = 2}, false);
    robot::chassis.moveToPoint(
        20, 6, 1800, {.forwards = false, .minSpeed = 70, .earlyExitRange = 2},
        false);
    robot::chassis.swingToPoint(28, 12, lemlib::DriveSide::LEFT, 1000,
                                {.minSpeed = 50, .earlyExitRange = 2}, false);
    robot::stickL.set_value(0);
    robot::chassis.moveToPoint(78, 24, 1300,
                               {.minSpeed = 60, .earlyExitRange = 2}, false);
    robot::leftMtrs.move(127);
    robot::rightMtrs.move(127);
    pros::delay(400);
    robot::leftMtrs.move(0);
    robot::rightMtrs.move(0);
  } else if (autonSelect == 5) {
    // Red 6+1+CORNER
    robot::chassis.setPose(0, -2.5, 0);
    pros::Task lbTask(_1028A::auton::lbTask);
    robot::LBS.set_position(100);
    robot::chassis.swingToHeading(-31, lemlib::DriveSide::LEFT, 400,
                                  {.minSpeed = 60, .earlyExitRange = 2}, false);
    armtarget = 580;
    pros::delay(500);
    robot::chassis.moveToPoint(11, -19, 2000,
                               {.forwards = false,
                                .maxSpeed = 95,
                                .minSpeed = 40,
                                .earlyExitRange = 3},
                               true);
    robot::chassis.waitUntil(6);
    reset = 1;
    robot::chassis.moveToPoint(10, -35, 2000,
                               {.forwards = false,
                                .maxSpeed = 95,
                                .minSpeed = 35,
                                .earlyExitRange = 3},
                               true);
    robot::chassis.waitUntil(28);
    robot::mogo.set_value(1);
    robot::chassis.waitUntilDone();
    pros::Task intakeTask(intakeControlTask);
    setIntakeState(IntakeState::COLOR_SORT_BLUE);
    // pros::Task intakeTask(intakeControlTask);
    // setIntakeState(IntakeState::COLOR_SORT_RED);
    robot::intake.move(127);
    robot::chassis.turnToPoint(41, -37.25, 700,
                               {.minSpeed = 50, .earlyExitRange = 2}, false);
    robot::chassis.moveToPoint(
        47.5, -39, 2000, {.maxSpeed = 95, .minSpeed = 55, .earlyExitRange = 11},
        false);
    pros::delay(100);
    robot::chassis.swingToPoint(-10, -25, lemlib::DriveSide::LEFT, 1000,
                                {.minSpeed = 80, .earlyExitRange = 4}, false);
    robot::leftMtrs.move(127);
    robot::rightMtrs.move(127);
    pros::delay(50);
    robot::leftMtrs.move(0);
    robot::rightMtrs.move(0);
    robot::chassis.turnToPoint(45, 4, 1000,
                               {.minSpeed = 0, .earlyExitRange = 4}, false);
    armtarget = 610;
    robot::chassis.moveToPoint(50, 8, 1000,
                               {.minSpeed = 50, .earlyExitRange = 2}, false);
    robot::leftMtrs.move(127);
    robot::rightMtrs.move(127);
    pros::delay(400);
    robot::leftMtrs.move(0);
    robot::rightMtrs.move(0);
    // robot::chassis.moveToPoint(-36, 6, 1000, {.forwards = false, .minSpeed =
    // 30}, false);
    pros::delay(100);
    robot::chassis.turnToHeading(-100, 1000, {}, false);
    robot::stickL.set_value(1);
    pros::delay(100);
    robot::chassis.moveToPoint(-4, 6, 1800,
                               {.minSpeed = 70, .earlyExitRange = 4}, false);
    robot::chassis.moveToPoint(-35, 6, 1800,
                               {.maxSpeed = 70, .earlyExitRange = 2}, false);
    robot::chassis.moveToPoint(
        -20, 6, 1800, {.forwards = false, .minSpeed = 70, .earlyExitRange = 2},
        false);
    robot::chassis.swingToPoint(-28, 12, lemlib::DriveSide::RIGHT, 1000,
                                {.minSpeed = 50, .earlyExitRange = 2}, false);
    robot::stickL.set_value(0);
    robot::chassis.moveToPoint(-78, 26.5, 1300,
                               {.minSpeed = 60, .earlyExitRange = 2}, false);
    robot::leftMtrs.move(127);
    robot::rightMtrs.move(127);
    pros::delay(400);
    robot::leftMtrs.move(0);
    robot::rightMtrs.move(0);
  } else if (autonSelect == 6) {
    // Blue 5+1+TOUCH
    robot::chassis.setPose(0, -2.5, 0);
    pros::Task lbTask(_1028A::auton::lbTask);
    robot::LBS.set_position(100);
    robot::chassis.swingToHeading(28, lemlib::DriveSide::RIGHT, 400,
                                  {.minSpeed = 60, .earlyExitRange = 2}, false);
    armtarget = 580;
    pros::delay(500);
    robot::chassis.moveToPoint(-11, -19, 2000,
                               {.forwards = false,
                                .maxSpeed = 95,
                                .minSpeed = 40,
                                .earlyExitRange = 3},
                               true);
    robot::chassis.waitUntil(6);
    reset = 1;
    robot::chassis.moveToPoint(-10, -35, 2000,
                               {.forwards = false,
                                .maxSpeed = 95,
                                .minSpeed = 35,
                                .earlyExitRange = 3},
                               true);
    robot::chassis.waitUntil(28);
    robot::mogo.set_value(1);
    robot::chassis.waitUntilDone();
    pros::Task intakeTask(intakeControlTask);
    setIntakeState(IntakeState::COLOR_SORT_RED);
    robot::intake.move(127);
    robot::chassis.turnToPoint(-41, -38, 700,
                               {.minSpeed = 50, .earlyExitRange = 2}, false);
    robot::chassis.moveToPoint(
        -46, -36.4, 2000,
        {.maxSpeed = 95, .minSpeed = 55, .earlyExitRange = 11}, false);
    pros::delay(100);
    robot::chassis.swingToPoint(10, -25, lemlib::DriveSide::RIGHT, 1000,
                                {.minSpeed = 80, .earlyExitRange = 4}, false);
    robot::leftMtrs.move(127);
    robot::rightMtrs.move(127);
    pros::delay(50);
    robot::leftMtrs.move(0);
    robot::rightMtrs.move(0);
    robot::chassis.turnToPoint(-45, 4, 1000,
                               {.minSpeed = 0, .earlyExitRange = 4}, false);
    robot::chassis.moveToPoint(-45.5, 8, 1000,
                               {.minSpeed = 50, .earlyExitRange = 2}, false);
    robot::leftMtrs.move(127);
    robot::rightMtrs.move(127);
    pros::delay(400);
    robot::leftMtrs.move(0);
    robot::rightMtrs.move(0);
    // robot::chassis.moveToPoint(-36, 6, 1000, {.forwards = false, .minSpeed =
    // 30}, false);
    pros::delay(100);
    robot::chassis.turnToHeading(100, 1000, {}, false);
    robot::stickL.set_value(1);
    pros::delay(100);
    robot::chassis.moveToPoint(4, 6, 1800,
                               {.minSpeed = 70, .earlyExitRange = 4}, false);
    robot::chassis.moveToPoint(23, 6, 1800,
                               {.maxSpeed = 70, .earlyExitRange = 2}, false);
    robot::stickL.set_value(0);
    robot::chassis.moveToPoint(
        20, 6, 1800, {.forwards = false, .minSpeed = 70, .earlyExitRange = 2},
        false);
    pros::delay(300);
    robot::chassis.moveToPoint(-5, -10, 1000,
                               {.forwards = false, .minSpeed = 50}, false);
    robot::intake.move(0);
    robot::chassis.turnToHeading(125, 1000, {}, false);
    // legacy::forward(15, NAN, 127, 1000, 1);
    pros::delay(100);
    robot::chassis.moveToPoint(-1.5, -23.5, 800, {}, false);

  } 
  else if (autonSelect == 7){
    //RED 6+1+Touch
    robot::chassis.setPose(0, -2.5, 0);
    pros::Task lbTask(_1028A::auton::lbTask);
    robot::LBS.set_position(100);
    robot::chassis.swingToHeading(-31, lemlib::DriveSide::LEFT, 400,
                                  {.minSpeed = 60, .earlyExitRange = 2}, false);
    armtarget = 580;
    pros::delay(500);
    robot::chassis.moveToPoint(11, -19, 2000,
                               {.forwards = false,
                                .maxSpeed = 95,
                                .minSpeed = 40,
                                .earlyExitRange = 3},
                               true);
    robot::chassis.waitUntil(6);
    reset = 1;
    robot::chassis.moveToPoint(10, -35, 2000,
                               {.forwards = false,
                                .maxSpeed = 95,
                                .minSpeed = 35,
                                .earlyExitRange = 3},
                               true);
    robot::chassis.waitUntil(28);
    robot::mogo.set_value(1);
    robot::chassis.waitUntilDone();
    pros::Task intakeTask(intakeControlTask);
    setIntakeState(IntakeState::COLOR_SORT_BLUE);
    // pros::Task intakeTask(intakeControlTask);
    // setIntakeState(IntakeState::COLOR_SORT_RED);
    robot::intake.move(127);
    robot::chassis.turnToPoint(41, -37.25, 700,
                               {.minSpeed = 50, .earlyExitRange = 2}, false);
    robot::chassis.moveToPoint(
        47.5, -39, 2000, {.maxSpeed = 95, .minSpeed = 55, .earlyExitRange = 11},
        false);
    pros::delay(100);
    robot::chassis.swingToPoint(-10, -25, lemlib::DriveSide::LEFT, 1000,
                                {.minSpeed = 80, .earlyExitRange = 4}, false);
    robot::leftMtrs.move(127);
    robot::rightMtrs.move(127);
    pros::delay(50);
    robot::leftMtrs.move(0);
    robot::rightMtrs.move(0);
    robot::chassis.turnToPoint(45, 4, 1000,
                               {.minSpeed = 0, .earlyExitRange = 4}, false);
    armtarget = 610;
    robot::chassis.moveToPoint(50, 8, 1000,
                               {.minSpeed = 50, .earlyExitRange = 2}, false);
    robot::leftMtrs.move(127);
    robot::rightMtrs.move(127);
    pros::delay(400);
    robot::leftMtrs.move(0);
    robot::rightMtrs.move(0);

    
    
  }else if (autonSelect == 8) {
    // RedGoalRings
    robot::chassis.setPose(0, -2.5, 0);
    pros::Task lbTask(_1028A::auton::lbTask);
    robot::LBS.set_position(100);
    robot::chassis.swingToHeading(26.5, lemlib::DriveSide::RIGHT, 400,
                                  {.minSpeed = 60, .earlyExitRange = 2}, false);
    armtarget = 580;
    pros::delay(500);
    robot::chassis.moveToPoint(-12, -28, 1300,
                               {.forwards = false,
                                .maxSpeed = 95,
                                .minSpeed = 40,
                                .earlyExitRange = 3},
                               true);
    pros::delay(100);
    reset = 1;
    robot::chassis.waitUntil(25);
    robot::mogo.set_value(1);
    robot::chassis.turnToHeading(112, 1000, {}, false);
    robot::chassis.moveToPose(30, -37, 90, 1000,
                              {.minSpeed = 55, .earlyExitRange = 5}, false);
    robot::chassis.moveToPoint(
        0, -31, 1000, {.forwards = false, .minSpeed = 40, .earlyExitRange = 2},
        false);
    robot::chassis.turnToHeading(163, 1000, {.earlyExitRange = 2}, false);
    robot::chassis.moveToPoint(2, -35, 1000,
                               {.minSpeed = 55, .earlyExitRange = 2}, false);

    robot::stickR.set_value(1);
    pros::delay(400);
    robot::chassis.moveToPoint(
        -16, -21, 1000,
        {.forwards = false, .minSpeed = 50, .earlyExitRange = 2}, false);
    pros::Task intakeTask(intakeControlTask);
    setIntakeState(IntakeState::COLOR_SORT_BLUE);
    pros::delay(100);
    robot::chassis.turnToHeading(-80, 1000, {.earlyExitRange = 5}, false);
    robot::stickR.set_value(0);
    robot::chassis.turnToHeading(-100, 1000, {.earlyExitRange = 5}, false);
    robot::chassis.moveToPoint(-34, -22, 1000,
                               {.minSpeed = 50, .earlyExitRange = 2}, false);
    robot::chassis.moveToPoint(-52, 7, 1000,
                               {.minSpeed = 50, .earlyExitRange = 2}, false);
    robot::leftMtrs.move(127);
    robot::rightMtrs.move(127);
    pros::delay(400);
    robot::leftMtrs.move(0);
    robot::rightMtrs.move(0);
    robot::chassis.turnToHeading(460, 1000, {}, false);
    robot::stickL.set_value(1);
    robot::chassis.moveToPoint(4, 4, 1000,
                               {.minSpeed = 50, .earlyExitRange = 2}, false);
    robot::chassis.turnToHeading(180, 1000, {.earlyExitRange = 5}, false);
    robot::chassis.moveToPoint(12, -7, 1000,
                               {.minSpeed = 50, .earlyExitRange = 2}, false);
  } else if (autonSelect == 8) {
    // BlueGoalRings
    robot::chassis.setPose(0, -2.5, 0);
    pros::Task lbTask(_1028A::auton::lbTask);
    robot::LBS.set_position(100);
    robot::chassis.swingToHeading(-28.5, lemlib::DriveSide::LEFT, 400,
                                  {.minSpeed = 60, .earlyExitRange = 2}, false);
    armtarget = 580;
    pros::delay(500);
    robot::chassis.moveToPoint(12, -33, 1800,
                               {.forwards = false,
                                .maxSpeed = 95,
                                .minSpeed = 40,
                                .earlyExitRange = 3},
                               true);
    pros::delay(100);
    reset = 1;
    robot::chassis.waitUntil(34);
    robot::mogo.set_value(1);
    robot::chassis.turnToHeading(-110, 1000, {}, false);
    robot::chassis.moveToPose(-14, -40, -90, 1000,
                              {.minSpeed = 65, .earlyExitRange = 5}, false);

    robot::chassis.moveToPoint(-4.5, -36, 1000,
                              {.forwards = false, .minSpeed = 40, .earlyExitRange = 2},
                              false);
    robot::chassis.turnToHeading(-146, 1000, {.earlyExitRange = 2}, false);
    robot::stickL.set_value(1);
    pros::delay(300);
    robot::chassis.turnToHeading(-110, 800, {.earlyExitRange = 2}, false);
    robot::chassis.moveToPoint(4, -21, 1000,
                               {.forwards = false, .minSpeed = 50, .earlyExitRange = 2},
                               false);

    // robot::chassis.moveToPoint(
    //     -5, -34, 1000, {.forwards = false, .minSpeed = 40, .earlyExitRange = 2},
    //     false);
    // robot::chassis.turnToHeading(-163, 1000, {.earlyExitRange = 2}, false);
    // robot::chassis.moveToPoint(-8., -40, 1000,
    //                            {.minSpeed = 55, .earlyExitRange = 2}, false);

    // robot::stickR.set_value(1);
    // pros::delay(400);
    // robot::chassis.moveToPoint(
    //     16, -21, 1000, {.forwards = false, .minSpeed = 50, .earlyExitRange = 2},
    //     false);
    // pros::Task intakeTask(intakeControlTask);
    // setIntakeState(IntakeState::COLOR_SORT_RED);
    // pros::delay(100);
    // robot::chassis.turnToHeading(80, 1000, {.earlyExitRange = 5}, false);
    // robot::stickR.set_value(0);
    // robot::chassis.turnToHeading(100, 1000, {.earlyExitRange = 5}, false);
    // robot::chassis.moveToPoint(34, -22, 1000,
    //                            {.minSpeed = 50, .earlyExitRange = 2}, false);
    // robot::chassis.moveToPoint(52, 7, 1000,
    //                            {.minSpeed = 50, .earlyExitRange = 2}, false);
    // robot::leftMtrs.move(127);
    // robot::rightMtrs.move(127);
    // pros::delay(400);
    // robot::leftMtrs.move(0);
    // robot::rightMtrs.move(0);
    // robot::chassis.turnToHeading(460, 1000, {}, false);
    // robot::stickL.set_value(1);
    // robot::chassis.moveToPoint(-4, 4, 1000,
    //                            {.minSpeed = 50, .earlyExitRange = 2}, false);
    // robot::chassis.turnToHeading(-180, 1000, {.earlyExitRange = 5}, false);
    // robot::chassis.moveToPoint(-12, -7, 1000,
    //                            {.minSpeed = 50, .earlyExitRange = 2}, false);
  } else if (autonSelect == 100) {
    // skills
    pros::Task odo(odomRead);
    pros::Task checkintake{checkIntake};
    robot::chassis.setPose(0, -3, 0);
    pros::Task lbTask(_1028A::auton::lbTask);
    robot::LBS.set_position(100);
    armtarget = 500;
    pros::delay(500);
    reset = 1;
    robot::chassis.moveToPose(30, -5, -108, 1800,
                              {.forwards = false, .minSpeed = 70}, false);
    robot::mogo.set_value(1);
    robot::chassis.waitUntilDone();
    robot::chassis.moveToPoint(28, -25, 1000,
                               {.minSpeed = 60, .earlyExitRange = 3}, true);
    robot::chassis.moveToPoint(42, -74, 1000,
                               {.minSpeed = 60, .earlyExitRange = 3}, true);
    robot::chassis.moveToPoint(52, -95, 1000,
                               {.minSpeed = 45, .earlyExitRange = 2}, true);
    checkintake.suspend();
    pros::delay(800);
    armtarget = 110;
    robot::chassis.waitUntilDone();
    pros::delay(300);
    robot::chassis.moveToPoint(
        35, -53.3, 2000, {.forwards = false, .maxSpeed = 70, .minSpeed = 40},
        false);
    pros::delay(200);
    robot::chassis.turnToHeading(-265, 1000, {}, false);
    robot::intake.move(-127);
    pros::delay(20);
    robot::intake.move(0);
    armtarget = 200;
    pros::delay(200);
    pros::Task queue(queueDisk);
    legacy::forward(19, -265, 80, 1000, 1);
    armtarget = 600;
    pros::delay(600);
    armtarget = 110;
    pros::delay(290);
    robot::intake.move(127);
    pros::delay(700);
    robot::intake.move(-127);
    pros::delay(20);
    robot::intake.move(0);
    armtarget = 580;
    pros::delay(550);
    legacy::forward(-12, NAN, 127, 1000, 1);
    robot::chassis.turnToHeading(-353, 500, {}, false);
    robot::intake.move(127);
    checkintake.resume();
    armtarget = 0;
    robot::chassis.moveToPose(
        48, 9, -354, 2500,
        {.maxSpeed = 80, .minSpeed = 50, .earlyExitRange = 3}, false);
    robot::chassis.turnToHeading(
        90, 800,
        {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE,
         .earlyExitRange = 3},
        false);
    robot::chassis.moveToPose(56, -13, -194, 2000,
                              {.minSpeed = 50, .earlyExitRange = 3}, false);
    robot::chassis.moveToPoint(65, 12, 800, {.forwards = false, .minSpeed = 50},
                               false);
    pros::delay(100);
    robot::mogo.set_value(0);
    robot::intake.move(-60);
    robot::chassis.turnToHeading(-135, 800, {}, false);
    robot::chassis.moveToPoint(39.5, 0, 800, {.minSpeed = 50}, false);
    robot::chassis.turnToHeading(90, 800, {}, false);
    robot::chassis.moveToPose(-36, -4, 90, 4000,
                              {.forwards = false,
                               .lead = 0.55,
                               .maxSpeed = 100,
                               .minSpeed = 40,
                               .earlyExitRange = 1},
                              false);
    robot::mogo.set_value(1);
    pros::delay(200);
    robot::intake.move(127);
    robot::chassis.moveToPoint(-30, -20, 1000,
                               {.minSpeed = 60, .earlyExitRange = 3}, true);
    robot::chassis.moveToPoint(-47, -64, 1000,
                               {.minSpeed = 50, .earlyExitRange = 6}, true);
    robot::chassis.moveToPoint(-67, -90.7, 1000,
                               {.earlyExitRange = 3}, true);
                              
    checkintake.suspend();
    pros::delay(850);
    armtarget = 110;
    robot::chassis.waitUntilDone();
    pros::delay(300);
    robot::chassis.moveToPoint(
      -48.5, -48.5, 2000, {.forwards = false, .maxSpeed = 70, .minSpeed = 40},
      false);
    pros::delay(200);
    robot::chassis.turnToHeading(-95, 1000, {}, false);
    robot::intake.move(-127);
    pros::delay(20);
    robot::intake.move(0);
    armtarget = 200;
    pros::delay(200);
    pros::Task quEUue(queueDisk);
    legacy::forward(19, -95, 80, 1000, 1);
    armtarget = 600;
    pros::delay(600);
    armtarget = 110;
    pros::delay(290);
    robot::intake.move(127);
    pros::delay(700);
    robot::intake.move(-127);
    pros::delay(20);
    robot::intake.move(0);
    armtarget = 580;
    pros::delay(550);
    legacy::forward(-12, NAN, 127, 1000, 1);
    robot::chassis.turnToHeading(0, 500, {}, false);
    robot::intake.move(127);
    checkintake.resume();
    pros::delay(100);
    robot::chassis.moveToPose(
      -54, 9, -354, 2500,
      {.maxSpeed = 80, .minSpeed = 50, .earlyExitRange = 3}, false);
    robot::chassis.turnToHeading(-90, 1000, {}, false);
    robot::chassis.moveToPoint(-62.5, -13, 2000,
                               {.minSpeed = 50, .earlyExitRange = 3}, false);
    robot::chassis.moveToPoint(-67, 10, 800, {.forwards = false, .minSpeed = 50},
                               false);
    pros::delay(100);
    robot::mogo.set_value(0);
    robot::intake.move(-20);
    pros::delay(100);
    robot::chassis.moveToPoint(-28, -68, 2000,
                               {.minSpeed = 60, .earlyExitRange = 3}, true);
    robot::chassis.waitUntil(20);
    robot::intake.move(127);
    checkintake.suspend();
    armtarget = 110;
    robot::chassis.waitUntilDone();
    robot::chassis.moveToPoint(-65, -112, 2500,
                               {.minSpeed = 60, .earlyExitRange = 3},
                               false);
  }
}