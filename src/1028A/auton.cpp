#include "1028A/auton.h"
#include "1028A/robot.h"
#include "lemlib/chassis/chassis.hpp"
#include "pros/rtos.hpp"
#include "1028A/logger.h"
#include "1028A/legacy.h"
#include <queue>
#include <cmath>

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

    if (reset){
      _1028A::robot::LB.move(127);
      pros::delay(200);
      _1028A::robot::LB.move(-40);
      pros::delay(200);
      while (1){
        if (fabs(_1028A::robot::LBS.get_velocity())<50){
          _1028A::robot::LBS.set_position(0);
          armtarget = 0;
          pros::delay(200);
          _1028A::robot::LB.move(0);
          reset = 0;
          break;
        }
        pros::delay(20);
      }
    }
    else{
      if (!armOverride or !auton::autonStop){
        _1028A::robot::LB.move(armSpeed);
      }
    }

    if (!pros::competition::is_autonomous() && !pros::competition::is_disabled()){
      break;
    }

    armPrevError = armError;

    pros::delay(10);
  }
}


enum intakeState{
    INTAKE,
    OUTTAKE,
    STOP,
    ColorSortRed,
    ColorSortBlue
};
intakeState intake = STOP;
void _1028A::auton::intakeTask(){
  bool kickout = false;
   const int timeThreshold = 1000;
  const int velocityThreshold = 5;
  int count = 0;
  int lastCount = 0;
  while (1){
    if (!pros::competition::is_autonomous() && !pros::competition::is_disabled() && pros::competition::is_connected()){
      break;
    }
    if (!autonStop){
    if (intake == INTAKE){
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
    }
    else if (intake == OUTTAKE){
      _1028A::robot::optical.set_led_pwm(0);
      _1028A::robot::intake.move(-127);
    }
    else if (intake == STOP){
      _1028A::robot::optical.set_led_pwm(0);
      _1028A::robot::intake.move(0);
    }
    else if (intake == ColorSortBlue){
      
    }
    else if (intake == ColorSortRed){
      
    }
  }
}
}

int Pause = 0;
void checkIntake(){
  const int timeThreshold = 500;
  const int velocityThreshold = 5;
  while (1){
    if (!pros::competition::is_autonomous() && !pros::competition::is_disabled() && pros::competition::is_connected()){
      break;
    }
    
    if (Pause != 1 or !_1028A::auton::autonStop){
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

void odomRead (){
    while (1){
        if (1){
            std::string data = "(" + std::to_string(_1028A::robot::chassis.getPose().x) + ", " + std::to_string(_1028A::robot::chassis.getPose().y) + ", " + std::to_string(_1028A::robot::chassis.getPose().theta) + ")";
            _1028A::logger::info(data.c_str());
        }
        pros::delay(400);
    }
}

void Disk(){
  _1028A::robot::intake.move(127);
  while (1){
    if (_1028A::robot::optical.get_proximity() > 253){
               _1028A::robot::intake.move(0);
               break;
            }
    pros::delay(20);
  }
}

void queueDisk(){
  _1028A::robot::intake.move(127);
  while (1){
    if (_1028A::robot::optical.get_proximity() > 253 or _1028A::robot::Ldistance.get() < 50){
               _1028A::robot::intake.move(0);
               break;
            }
    pros::delay(5);
  }
}
enum class DiskColor {
  NONE,
  BLUE,
  RED,
  UNKNOWN
};

struct DiskInfo {
  DiskColor color;
  uint32_t  entryTime;
};

enum class IntakeState {
  INTAKE,
  EXPEL,
  COLOR_SORT_BLUE,
  COLOR_SORT_RED,
  STOP
};

static IntakeState g_currentState = IntakeState::STOP;
static bool g_lastSawDisc = false;
static std::queue<DiskInfo> g_diskQueue;
static bool     g_jamTimerActive = false;
static uint32_t g_jamStartTime   = 0;
static bool     g_seenTopDisc         = false;
static uint32_t g_topDiscFirstSeenTime = 0;
constexpr int    INTAKE_SPEED          = 127;
constexpr int    EXPEL_SPEED           = 127;
constexpr int    REVERSE_SPEED         = 127;
constexpr int    EJECT_TIME_MS         = 300;
constexpr double TOP_DETECTION_THRESH  = 50.0;
constexpr int    COLOR_SORT_DELAY_MS   = 0; 
constexpr double JAM_VELOCITY_DIFF     = 50.0;
constexpr int    JAM_TIME_THRESHOLD_MS = 500;  
constexpr int    DEJAM_REVERSE_TIME_MS = 300;  
constexpr double BLUE_HUE_MIN = 110.0;
constexpr double BLUE_HUE_MAX = 300.0;
constexpr double RED_HUE_MIN  =   0.0;
constexpr double RED_HUE_MAX  =  90.0;
constexpr uint32_t DISC_TIMEOUT_MS     = 600;
constexpr uint32_t SECOND_SENSOR_DELAY_MS = 200;

void setIntakeState(IntakeState newState) {
  g_currentState = newState;
}

static DiskColor getDiskColor() {
  double hue1 = _1028A::robot::optical.get_hue();

  if (hue1 >= BLUE_HUE_MIN && hue1 <= BLUE_HUE_MAX) {
    return DiskColor::BLUE;
  }
  else if ((hue1 >= RED_HUE_MIN && hue1 <= RED_HUE_MAX)
        || (hue1 >= 330.0 && hue1 <= 360.0)) {
    return DiskColor::RED;
  }

  // Fallback to second sensor
  double hue2 = _1028A::robot::opticalH.get_hue();

  if (hue2 >= BLUE_HUE_MIN && hue2 <= BLUE_HUE_MAX) {
    std::cout << "[DEBUG] Second sensor used: Detected BLUE\n";
    return DiskColor::BLUE;
  }
  else if ((hue2 >= RED_HUE_MIN && hue2 <= RED_HUE_MAX)
        || (hue2 >= 330.0 && hue2 <= 360.0)) {
    std::cout << "[DEBUG] Second sensor used: Detected RED\n";
    return DiskColor::RED;
  }

  std::cout << "[DEBUG] Second sensor used: Still UNKNOWN\n";
  return DiskColor::UNKNOWN;
}

void tryUpdateUnknownDiskColor() {
  if (g_diskQueue.empty()) return;

  DiskInfo& frontDisk = g_diskQueue.front();

  if (frontDisk.color == DiskColor::UNKNOWN &&
      pros::millis() - frontDisk.entryTime >= SECOND_SENSOR_DELAY_MS) {
    double hue2 = _1028A::robot::opticalH.get_hue();

    if (hue2 >= BLUE_HUE_MIN && hue2 <= BLUE_HUE_MAX) {
      frontDisk.color = DiskColor::BLUE;
      std::cout << "[DEBUG] Updated UNKNOWN to BLUE via second sensor\n";
    } 
    else if ((hue2 >= RED_HUE_MIN && hue2 <= RED_HUE_MAX)
          || (hue2 >= 330.0 && hue2 <= 360.0)) {
      frontDisk.color = DiskColor::RED;
      std::cout << "[DEBUG] Updated UNKNOWN to RED via second sensor\n";
    }
  }
}

static const char* colorToString(DiskColor c) {
  switch (c) {
    case DiskColor::BLUE:    return "BLUE";
    case DiskColor::RED:     return "RED";
    case DiskColor::UNKNOWN: return "UNKNOWN";
    default:                 return "NONE";
  }
}

void intakeControlTask(void* param) {
  _1028A::robot::optical.set_led_pwm(100);
  _1028A::robot::opticalH.set_led_pwm(100);

  while (true) {
    if (!pros::competition::is_autonomous() && !pros::competition::is_disabled()) {
      break;
    }

    bool seesDisc = (_1028A::robot::optical.get_proximity() > 200); 
    if (seesDisc && !g_lastSawDisc) {
      DiskColor c = getDiskColor();
      DiskInfo newDisk { c, pros::millis() };
      g_diskQueue.push(newDisk);

      std::cout << "[DEBUG] ** New Disc Detected **  Color=" 
                << colorToString(c) 
                << "  entryTime=" << newDisk.entryTime << "\n";
    }
    g_lastSawDisc = seesDisc;

    tryUpdateUnknownDiskColor();

    int targetVelocity = 0;
    int direction      = 1;

    switch (g_currentState) {
      case IntakeState::INTAKE:
        targetVelocity = INTAKE_SPEED;
        direction      = +1;
        break;
      case IntakeState::EXPEL:
        targetVelocity = EXPEL_SPEED;
        direction      = -1;
        break;
      case IntakeState::COLOR_SORT_BLUE:
      case IntakeState::COLOR_SORT_RED:
        targetVelocity = INTAKE_SPEED; 
        direction      = +1;
        break;
      case IntakeState::STOP:
      default:
        targetVelocity = 0;
        direction      = 0;
        break;
    }
    _1028A::robot::intake.move(targetVelocity * direction);

    if (g_currentState == IntakeState::COLOR_SORT_BLUE
     || g_currentState == IntakeState::COLOR_SORT_RED) 
    {
      double topDist    = _1028A::robot::distance.get();
      bool   topHasDisc = (topDist < TOP_DETECTION_THRESH);

      if (topHasDisc && !g_seenTopDisc && !g_diskQueue.empty()) {
        g_seenTopDisc          = true;
        g_topDiscFirstSeenTime = pros::millis();

        std::cout << "[DEBUG] Top disc first seen at time=" 
                  << g_topDiscFirstSeenTime << "\n";
      }

      if (!topHasDisc) {
        g_seenTopDisc = false;
      }

      if (g_seenTopDisc 
          && (pros::millis() - g_topDiscFirstSeenTime >= COLOR_SORT_DELAY_MS)
          && !g_diskQueue.empty()) 
      {
        DiskColor colorToEject = (g_currentState == IntakeState::COLOR_SORT_BLUE)
                                   ? DiskColor::BLUE
                                   : DiskColor::RED;

        DiskInfo frontInfo = g_diskQueue.front(); 
        std::cout << "[DEBUG] Checking front disc: color=" 
                  << colorToString(frontInfo.color)
                  << "  in queue for=" 
                  << (pros::millis() - frontInfo.entryTime) << " ms\n";

        if (frontInfo.color == colorToEject || frontInfo.color == DiskColor::UNKNOWN) {
          std::cout << "[DEBUG] --> Ejecting disc color=" 
                    << colorToString(frontInfo.color) << "\n";

          pros::delay(80);
          _1028A::robot::intake.move(-EXPEL_SPEED);
          pros::delay(EJECT_TIME_MS);

          _1028A::robot::intake.move(targetVelocity);

          g_diskQueue.pop();
        } else {
          std::cout << "[DEBUG] --> Disc color doesn't match target; not ejecting.\n";
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

void _1028A::auton::auton(){
  autonSelect = 8;
  robot::leftMtrs.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
  robot::rightMtrs.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);

if (autonSelect == 1){
  //redRingRush
  robot::chassis.setPose(0,0,0);
  pros::Task disk(queueDisk);
  robot::chassis.moveToPoint(-13, 40, 2000, {.earlyExitRange = 1.5}, false);
  pros::delay(300);
  robot::chassis.moveToPoint(7, 24, 2000, {.forwards = false}, true);
  robot::chassis.waitUntil(27);
  robot::mogo.set_value(1);
  robot::chassis.waitUntilDone();
  pros::delay(50);
  robot::chassis.turnToPoint(-16, 28, 500, {.earlyExitRange = 2}, false);
  robot::chassis.moveToPoint(-16, 25, 2000, {.earlyExitRange = 2}, true);
  robot::intake.move(127);
  robot::chassis.waitUntilDone();
  robot::chassis.moveToPoint(-26, 35, 2000, {.earlyExitRange = 2}, true);
}
else if (autonSelect == 3){
  //Red Rush
  robot::stickL.set_value(1);
  robot::intake.move(127);
  pros::Task queued(queueDisk);
    robot::chassis.moveToPoint(0, 34, 2000, {.minSpeed=80}, false);
    robot::stickL.set_value(0);
    robot::chassis.moveToPoint(0, 15, 2000, {.forwards = false}, true);
    robot::chassis.waitUntil(9);
    robot::stickL.set_value(1);
    robot::chassis.turnToHeading(-260, 1000, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .earlyExitRange = 4}, false);
    robot::stickL.set_value(0);
    robot::chassis.moveToPose(-28, 40.5, -220, 2000, {.forwards = false, .minSpeed=50}, false);
    robot::mogo.set_value(1);
    pros::delay(100);
  robot::intake.move(127);
  robot::chassis.turnToHeading(187, 1000, {.earlyExitRange = 3}, false);
  robot::chassis.moveToPose(-7.5, -5, 102, 2000, {.earlyExitRange = 2}, false);
  robot::leftMtrs.move(50);
  robot::rightMtrs.move(50);
  pros::delay(1700);
  robot::leftMtrs.move(0);
  robot::rightMtrs.move(0);
    

}
else if (autonSelect == 4){
  //Blue Rush
  robot::stickL.set_value(1);
  robot::intake.move(127);
  pros::Task queued(queueDisk);
    robot::chassis.moveToPoint(0, 34, 2000, {.minSpeed=80}, false);
    robot::stickL.set_value(0);
    robot::chassis.moveToPoint(0, 15, 2000, {.forwards = false}, true);
    robot::chassis.waitUntil(9);
    robot::stickL.set_value(1);
    robot::chassis.turnToHeading(260, 1000, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .earlyExitRange = 4}, false);
    robot::stickL.set_value(0);
    robot::chassis.moveToPoint(27, 27, 2000, {.forwards = false, .minSpeed=50}, false);
    robot::mogo.set_value(1);
    robot::chassis.turnToHeading(187, 1000, {.earlyExitRange = 3}, false); 
    robot::intake.move(127);
    robot::chassis.moveToPose(-11, -3, 280, 2000, {.minSpeed = 40, .earlyExitRange = 2}, false);
    robot::leftMtrs.move(50);
  robot::rightMtrs.move(50);
  pros::delay(1700);
  robot::leftMtrs.move(0);
  robot::rightMtrs.move(0);

}
else if (autonSelect == 5){
  //blue5Ring
  pros::Task lbTask(_1028A::auton::lbTask);
  pros::Task odo(odomRead);
  armtarget = 500;
  robot::chassis.moveToPoint(0,1.5, 2000, {.minSpeed = 55}, false);
  pros::delay(200);
  robot::chassis.moveToPose(24, -27, -74, 2000, {.forwards = false, .minSpeed = 60}, true);
  robot::chassis.waitUntil(43);
  robot::mogo.set_value(1);
  reset = 1;
  robot::chassis.waitUntilDone();
  pros::delay(200);
  robot::intake.move(127);
  pros::Task intakeTask(intakeControlTask);
  setIntakeState(IntakeState::COLOR_SORT_RED);
  robot::chassis.turnToHeading(-182.5, 500, {}, false);
  robot::chassis.moveToPose(0, -59, -132, 1400, {.minSpeed = 60, .earlyExitRange = 6}, false);
  robot::chassis.turnToHeading(-150, 800, {}, false);
  robot::chassis.moveToPoint(16, -28, 1000, {.forwards = false, .minSpeed = 60}, false);
  robot::chassis.turnToHeading(-124, 500, {}, false);
  robot::chassis.moveToPoint(-3, -40, 1000, {.minSpeed = 60}, false);
  robot::chassis.moveToPoint(10, -35, 1000, {.forwards = false, .minSpeed = 60}, false);
  armtarget = 610;
  robot::chassis.moveToPose(-44, -19.5, -45, 3000, {.minSpeed = 60}, false);
  robot::leftMtrs.move(127);
  robot::rightMtrs.move(127);
  pros::delay(600);
  robot::leftMtrs.move(0);
  robot::rightMtrs.move(0);
  robot::chassis.turnToPoint(17, -15, 1000, {}, false);
  robot::chassis.moveToPoint(20, -20, 1300, {}, true);
  pros::delay(500);
  armtarget = 0;
  robot::chassis.waitUntilDone();
  lbTask.suspend();

}

else if (autonSelect == 6){
  //red5Ring
  //pros::delay(2000);
 pros::Task lbTask(_1028A::auton::lbTask);
  pros::Task odo(odomRead);
  armtarget = 500;
  robot::chassis.moveToPoint(0,1.5, 2000, {.minSpeed = 55}, false);
  pros::delay(200);
  robot::chassis.moveToPose(-24, -27, 74, 2000, {.forwards = false, .minSpeed = 60}, true);
  robot::chassis.waitUntil(43);
  robot::mogo.set_value(1);
  reset = 1;
  robot::chassis.waitUntilDone();
  pros::delay(200);
  robot::intake.move(127);
  pros::Task intakeTask(intakeControlTask);
  setIntakeState(IntakeState::COLOR_SORT_BLUE);
  robot::chassis.turnToHeading(182.5, 500, {}, false);
  robot::chassis.moveToPose(1.5, -57, 130, 1000, {.minSpeed = 60, .earlyExitRange = 6}, false);
  robot::chassis.turnToHeading(150, 800, {}, false);
  robot::chassis.moveToPoint(-16, -28, 1000, {.forwards = false, .minSpeed = 60}, false);
  robot::chassis.turnToHeading(124, 500, {}, false);
  robot::chassis.moveToPoint(3, -40, 1000, {.minSpeed = 60}, false);
  robot::chassis.moveToPoint(-10, -35, 1000, {.forwards = false, .minSpeed = 60}, false);
  armtarget = 610;
  robot::chassis.moveToPose(44, -19.5, 45, 3000, {.minSpeed = 60}, false);
  robot::leftMtrs.move(127);
  robot::rightMtrs.move(127);
  pros::delay(600);
  robot::leftMtrs.move(0);
  robot::rightMtrs.move(0);
  robot::chassis.turnToPoint(-17, -15, 1000, {}, false);
  robot::chassis.moveToPoint(-19, -15, 1300, {}, true);
  pros::delay(500);
  armtarget = 0;
  robot::chassis.waitUntilDone();
  lbTask.suspend();
  // robot::chassis.turnToPoint(-28, 10, 1000, {.direction = lemlib::AngularDirection::CW_CLOCKWISE}, false);
  // robot::chassis.moveToPoint(-28, 10, 2000, {}, false);

  



  // robot::chassis.moveToPoint(-2, -33, 1000, {.minSpeed = 45}, true);
  // robot::chassis.moveToPoint(31, -17, 2000, {.minSpeed = 60}, false);
  // robot::leftMtrs.move(127);
  // robot::rightMtrs.move(127);
  // pros::delay(800);
  // robot::leftMtrs.move(0);
  // robot::rightMtrs.move(0);
  // robot::chassis.turnToPoint(-36, -5, 1000, {.direction=lemlib::AngularDirection::CW_CLOCKWISE}, false);
  // robot::chassis.moveToPoint(-36, 0, 1000, {}, false);
  // robot::chassis.moveToPose(7.2, -37.7, 107, 2000, {.minSpeed = 55}, false);
  // robot::chassis.moveToPoint(robot::chassis.getPose().x - 4, robot::chassis.getPose().y, 700, {.minSpeed = 50}, false);
  // lbTask.suspend();
  /*
  robot::chassis.turnToHeading(90, 500, {}, false);
  pros::delay(200);
  robot::chassis.moveToPose(34, -25, -140, 2300, {.minSpeed = 50}, false);
  robot::leftMtrs.move(75);
  robot::rightMtrs.move(75);
  pros::delay(500);
  robot::leftMtrs.move(0);
  robot::rightMtrs.move(0);
  pros::delay(300);
  */
  // robot::chassis.turnToHeading(-70, 800, {}, false);
  // robot::intake.move(0);
  // checkintake.suspend();
  // robot::chassis.moveToPose(-27, -15, -70, 3000, {}, true);
  // pros::delay(1300);
  // robot::chassis.waitUntilDone();

}
else if (autonSelect == 7){
   //redRing4+1;
  robot::chassis.setPose(0,0,0);
  robot::stickL.set_value(1);
  robot::chassis.moveToPoint(-11, 41.5, 1000, {.earlyExitRange = 1}, true);
  robot::LBS.set_position(0);
  pros::delay(400);
  pros::Task disk(queueDisk);
  robot::chassis.waitUntilDone();
  pros::delay(300);
  robot::intake.move(0);
  robot::chassis.moveToPoint(10, 29, 800, {.forwards = false, .earlyExitRange = 1.5}, true);
  robot::chassis.waitUntil(23);
  // robot::mogo.set_value(1);
  // robot::stickL.set_value(0);
  // robot::chassis.waitUntilDone();
  // robot::chassis.moveToPoint(14, 43, 1200, {.maxSpeed = 80}, true);
  // robot::chassis.waitUntil(25);
  // robot::stickL.set_value(1);
  // robot::leftMtrs.move(50);
  // robot::rightMtrs.move(50);
  // pros::delay(100);
  // robot::leftMtrs.move(0);
  // robot::rightMtrs.move(0);
  // robot::leftMtrs.move(-80);
  // robot::rightMtrs.move(-80);
  // pros::delay(300);
  // robot::leftMtrs.move(0);
  // robot::rightMtrs.move(0);
  // pros::delay(200);



  //robot::chassis.moveToPoint(11, 14, 1000, {.forwards = false, .maxSpeed = 60, .earlyExitRange = 2}, false);
  //robot::stickL.set_value(0);
  //robot::chassis.moveToPoint(14, 13, 1000, {.forwards = false, .earlyExitRange = 2}, true);
  //robot::chassis.turnToPoint(11, 40.5, 800, {.earlyExitRange = 1}, false);
  


  // robot::chassis.turnToPoint(-14,44, 500, {.earlyExitRange = 4}, false);
  // robot::chassis.moveToPoint(-14, 44, 900, {.earlyExitRange = 4}, true);
  // pros::Task intakeTask(intakeControlTask);
  // setIntakeState(IntakeState::COLOR_SORT_BLUE);
  // robot::chassis.waitUntilDone();
  // robot::chassis.turnToPoint(robot::chassis.getPose().x - 8, robot::chassis.getPose().y, 500, {.earlyExitRange = 2}, false);
  // robot::chassis.moveToPoint(robot::chassis.getPose().x - 15, robot::chassis.getPose().y, 1000, {.earlyExitRange = 2}, false);
  // robot::chassis.moveToPoint(5, 29, 800, {.forwards = false, .earlyExitRange = 2}, false);
  // robot::chassis.moveToPose(-23,18, -143, 1000, {.minSpeed = 80, .earlyExitRange = 2}, false);
  // robot::chassis.moveToPose(-37, -15, -162, 2000, {.minSpeed = 80, .earlyExitRange = 2}, false);
  // robot::leftMtrs.move(127);
  // robot::rightMtrs.move(127);
  // pros::delay(300);
  // robot::leftMtrs.move(0);
  // robot::rightMtrs.move(0);
  // robot::chassis.turnToPoint(0,-5,800, {.minSpeed = 50}, false);
  // robot::chassis.moveToPoint(0, -3, 1500, {.maxSpeed = 100, .earlyExitRange = 2,}, false);
  // robot::chassis.turnToPoint(16, -14.5, 800, {}, false);
  // intakeTask.suspend();
  // robot::intake.move(0);
  // robot::chassis.moveToPoint(16, -16, 800, {} ,false);
  // //robot::chassis.turnToHeading(-245, 400, {}, false);
  // robot::intake.move(-127);
  // pros::delay(20);
  // robot::intake.move(0);
  // armtarget = 550;
  // pros::delay(400);
  // robot::chassis.moveToPoint(0, -2, 500, {.forwards = false, .earlyExitRange = 5}, true);
  // lbTask.suspend();
  // intakeTask.resume();
  // setIntakeState(IntakeState::COLOR_SORT_BLUE);
  // robot::chassis.waitUntilDone();
  // robot::chassis.moveToPoint(40, 4, 1000, {.earlyExitRange = 2}, false);
}
else if (autonSelect == 8){
  //redSoloSigAWP
  
  pros::Task checkintake{checkIntake};
  pros::Task lbTask(_1028A::auton::lbTask);
  robot::LBS.set_position(100);
  armtarget = 520;
  robot::chassis.moveToPoint(0, 5, 500, {}, false);
  robot::chassis.moveToPose(-22, -24.5, 60, 3000, {.forwards = false, .minSpeed = 80}, true);
  robot::chassis.waitUntil(45);
  robot::mogo.set_value(1);
  robot::chassis.waitUntilDone();
  robot::chassis.moveToPoint(8, -28, 1000, {.earlyExitRange = 2}, false);
  pros::Task intakeTask(intakeControlTask);
  setIntakeState(IntakeState::COLOR_SORT_BLUE);
  //legacy::forward(-12, NAN, 127, 1000, 1);
  robot::chassis.moveToPoint(40, -21, 2000, {.minSpeed = 60}, false);
  robot::leftMtrs.move(127);
  robot::rightMtrs.move(127);
  pros::delay(300);
  robot::leftMtrs.move(0);
  robot::rightMtrs.move(0);
  robot::chassis.turnToPoint(-22, 30, 800, {.earlyExitRange = 2}, false);
  robot::chassis.moveToPoint(-32, 42, 4000, {.maxSpeed = 60, .earlyExitRange = 2}, true);
  reset = 1;
  robot::chassis.waitUntil(40);
  robot::mogo.set_value(0);
  intakeTask.suspend();
  checkintake.suspend();
  int St = pros::millis();
  robot::intake.move(-127);
  pros::delay(100);
  robot::intake.move(0);
  pros::delay(1000);
  while (1){
    if (pros::millis() - St > 2000){
      break;
    }
    else if (robot::distance.get() < 50){
      break;
    }
    pros::delay(20);
  }
  pros::Task disk(queueDisk);
  robot::chassis.waitUntilDone();
  robot::chassis.turnToHeading(23, 800, {}, false);
  legacy::forward(-29, NAN, 127, 1000, 1);
  robot::mogo.set_value(1);
  pros::delay(100);
  robot::chassis.turnToPoint(-58, 42, 800, {}, false);
  robot::intake.move(127);
  robot::chassis.moveToPoint(-58, 42, 1000, {}, false);
  robot::chassis.turnToPoint(-41, 15, 800, {}, false);
  robot::chassis.moveToPoint(-49, 14, 1000, {.maxSpeed = 80}, false);
}
else if (autonSelect == 9){
  //redRing3+1+touch
  pros::Task checkintake{checkIntake};
  pros::Task lbTask(_1028A::auton::lbTask);
  robot::LBS.set_position(100);
  armtarget = 520;
  robot::chassis.moveToPoint(0, 9, 500, {}, false);
  robot::chassis.moveToPoint(-11, -30, 1000, {.forwards = false}, true);
  robot::chassis.waitUntil(44);
  robot::mogo.set_value(1);
}
else if (autonSelect == 10){
  robot::chassis.moveToPoint(12, 37.5, 1500, {}, true);
  robot::chassis.waitUntil(25);
  pros::Task lbTask(_1028A::auton::lbTask);
  robot::LBS.set_position(100);
  armtarget = 600;

}
else if (autonSelect == 11){
  pros::Task lbTask(_1028A::auton::lbTask);
  robot::LBS.set_position(100);
  armtarget = 520;
  robot::chassis.moveToPoint(0, 5.5, 500, {}, false);
  pros::delay(300);
  robot::chassis.moveToPose(-24, -29, 74, 3000, {.forwards = false, .minSpeed = 60}, true);
  robot::chassis.waitUntil(42);
  robot::mogo.set_value(1);
  robot::chassis.waitUntilDone();
  robot::chassis.moveToPoint(-17, -45.5, 1400, {}, true);
  robot::chassis.waitUntil(3);
  robot::intake.move(127);
  robot::stickL.set_value(1);
  robot::chassis.waitUntilDone();
  pros::delay(200);
  robot::chassis.moveToPoint(-23, -25, 800, {.forwards = false}, true);
  robot::chassis.waitUntil(20);
  robot::stickL.set_value(0);
  robot::chassis.waitUntilDone();
  pros::delay(100);
  pros::Task intakeTask(intakeControlTask);
  setIntakeState(IntakeState::COLOR_SORT_BLUE);
  robot::chassis.moveToPoint(-2, -41, 1000, {.minSpeed = 45}, true);
  robot::chassis.moveToPoint(31, -22, 2000, {.minSpeed = 60}, false);
  robot::leftMtrs.move(127);
  robot::rightMtrs.move(127);
  pros::delay(600);
  robot::leftMtrs.move(0);
  robot::rightMtrs.move(0);
  robot::chassis.turnToPoint(-36, -27, 1000, {}, false);
  pros::delay(100);
  robot::intake.move(0);
  robot::chassis.moveToPoint(-25, -20,  1300, {.minSpeed = 30}, false);
  robot::stickL.set_value(0);
  lbTask.suspend();
  // robot::chassis.moveToPoint(-25, -32, 2000, {}, true);
  // robot::chassis.waitUntil(20);
  // robot::stickL.set_value(1);
  // pros::Task disk(queueDisk);
  // robot::chassis.waitUntilDone();
}
else if (autonSelect == 12){
  pros::Task lbTask(_1028A::auton::lbTask);
  robot::LBS.set_position(100);
  armtarget = 520;
  robot::chassis.moveToPoint(0, 5.5, 500, {}, false);
  pros::delay(300);
  pros::Task checkintake{checkIntake};
  robot::chassis.moveToPose(24, -29, -74, 3000, {.forwards = false, .minSpeed = 60}, true);
  pros::delay(100);
  reset = 1;
  robot::chassis.waitUntil(50);
  robot::mogo.set_value(1);
  robot::chassis.waitUntilDone();
  robot::chassis.moveToPoint(17, -44.5, 1400, {}, true);
  robot::chassis.waitUntil(3);
  robot::intake.move(127);
  robot::stickR.set_value(1);
  robot::chassis.waitUntilDone();
  pros::delay(200);
  robot::chassis.moveToPoint(23, -24, 800, {.forwards = false}, true);
  robot::chassis.waitUntil(15);
  robot::stickR.set_value(0);
  robot::chassis.waitUntilDone();
  pros::delay(100);
  checkintake.suspend();
  pros::Task intakeTask(intakeControlTask);
  setIntakeState(IntakeState::COLOR_SORT_RED);
  robot::chassis.moveToPoint(2, -38, 1000, {.minSpeed = 45}, true);
  robot::chassis.moveToPoint(-31, -22, 2000, {.minSpeed = 60}, false);
  robot::leftMtrs.move(127);
  robot::rightMtrs.move(127);
  pros::delay(300);
  robot::leftMtrs.move(0);
  robot::rightMtrs.move(0);
  robot::chassis.turnToPoint(36, -27, 1000, {}, false);
  pros::delay(100);
  robot::intake.move(0);
  robot::chassis.moveToPoint(25, -20,  1300, {.minSpeed = 30}, false);
  robot::stickL.set_value(0);
  lbTask.suspend();
  // robot::chassis.moveToPoint(-25, -32, 2000, {}, true);
  // robot::chassis.waitUntil(20);
  // robot::stickL.set_value(1);
  // pros::Task disk(queueDisk);
  // robot::chassis.waitUntilDone();
}
else if (autonSelect == 14){
  //RedRingRush
  robot::stickR.set_value(1);
  pros::Task que (queueDisk);
  robot::chassis.moveToPoint(-1, 38, 1500, {}, false);
  robot::leftMtrs.move(-70);
  robot::rightMtrs.move(-70);
  pros::delay(300);
  robot::leftMtrs.move(0);
  robot::rightMtrs.move(0);
  //robot::chassis.moveToPose(1.5, 40, -34, 1000, {.forwards = false, .minSpeed = 60, .earlyExitRange = 4}, false);
  robot::chassis.moveToPose(30, 30, -89, 1800, {.forwards = false, .maxSpeed = 60, .minSpeed = 20}, false);
  robot::stickR.set_value(0);
  robot::mogo.set_value(1);
  pros::Task intakeTask(intakeControlTask);
  setIntakeState(IntakeState::COLOR_SORT_BLUE);
  robot::chassis.moveToPoint(0, 24, 1400, {.maxSpeed = 70, .earlyExitRange = 4}, false);
  robot::chassis.moveToPoint(-11, -4, 1200, {}, false);
  robot::leftMtrs.move(127);
  robot::rightMtrs.move(127);
  pros::delay(300);
  robot::leftMtrs.move(0);
  robot::rightMtrs.move(0);
  robot::chassis.turnToHeading(-208, 700, {}, false);
  robot::chassis.moveToPose(42, 2, -306, 1300, {}, false);
  robot::stickR.set_value(1);
  robot::chassis.moveToPoint(16, -2, 1000, {.forwards = false}, false);
  

  
}
else if (autonSelect == 15){
  //RedGoalSafe
  pros::Task lbTask(_1028A::auton::lbTask);
  robot::LBS.set_position(100);
  armtarget = 520;
  robot::chassis.moveToPoint(0, 3, 400, {}, false);
  pros::delay(300);
  pros::Task intakeTask(intakeControlTask);
  setIntakeState(IntakeState::COLOR_SORT_BLUE);
  robot::chassis.moveToPose(24, -29, -74, 3000, {.forwards = false, .minSpeed = 60}, true);
  pros::delay(100);
  reset = 1;
  robot::chassis.waitUntil(50);
  robot::mogo.set_value(1);
  pros::delay(200);
  robot::chassis.moveToPoint(-1, -45, 1000, {.maxSpeed = 80, .earlyExitRange = 12}, true);
  armtarget = 610;
  pros::delay(200);
  robot::chassis.waitUntilDone();
  robot::chassis.moveToPoint(-29.5, -28, 2000, {.maxSpeed = 90, .minSpeed = 60}, false);
  robot::leftMtrs.move(127);
  robot::rightMtrs.move(127);
  pros::delay(300);
  robot::leftMtrs.move(0);
  robot::rightMtrs.move(0);
  robot::chassis.turnToPoint(36, -30, 1000, {}, true);
  pros::delay(900);
  setIntakeState(IntakeState::STOP);
  armtarget = 0; 
  robot::chassis.moveToPose(43.75, -22, 95,  5000, {.maxSpeed = 60, .minSpeed = 40}, true);
  robot::chassis.waitUntil(40);
  robot::stickR.set_value(1);
  robot::chassis.waitUntilDone();
  robot::chassis.moveToPose(9.5, -22.7, 95, 2500, {.forwards = false, .minSpeed = 70}, false);
  robot::stickR.set_value(0);
  setIntakeState(IntakeState::COLOR_SORT_BLUE);
  pros::delay(500);
  robot::chassis.moveToPoint(26, -29, 1000, {}, true);

}
else if (autonSelect == 16){
  //GoalScoreRushRed
  robot::chassis.moveToPose(5.2, 38.5, 23, 1900, {.minSpeed = 100}, true);
  robot::chassis.waitUntil(25);
  pros::Task lbTask(_1028A::auton::lbTask);
  armtarget = 720;
  robot::chassis.turnToHeading(32, 1000, {}, false);
  pros::delay(200);
  robot::chassis.moveToPoint(-14, 32, 1500, {.forwards = false, .minSpeed = 70}, true);
  robot::chassis.waitUntil(30);
  robot::mogo.set_value(1);
  pros::Task checkINTAke(checkIntake);
  pros::delay(800);
  checkINTAke.suspend();
  pros::Task intakeTask(intakeControlTask);
  setIntakeState(IntakeState::COLOR_SORT_BLUE);
  robot::chassis.moveToPose(13, 34, 80, 1500, {.minSpeed = 70}, false);
  pros::delay(300);
  robot::chassis.turnToPoint(50, 8, 1000, {}, false);
  robot::chassis.moveToPoint(24, 0, 1000, {}, true);
  robot::chassis.waitUntil(10);
  armtarget = 15;
  robot::leftMtrs.move(127);
  robot::rightMtrs.move(127);
  pros::delay(1100);
  robot::leftMtrs.move(0);
  robot::rightMtrs.move(0);
  robot::leftMtrs.move(-127);
  robot::rightMtrs.move(-127);
  pros::delay(200);
  robot::leftMtrs.move(0);
  robot::rightMtrs.move(0);
  robot::chassis.turnToPoint(33, 65, 1000, {}, false);
  robot::chassis.moveToPoint(26, 38.25, 3000, {.maxSpeed = 80, .earlyExitRange = 0.5}, true);
  robot::chassis.waitUntil(30);
  intakeTask.suspend();
  robot::intake.move(-127);
  pros::delay(20);
  robot::intake.move(0);
  robot::chassis.turnToHeading(44, 1000, {}, true);
  pros::delay(800);
  armtarget = 330;
  robot::leftMtrs.move(90);
  robot::rightMtrs.move(90);
  pros::delay(400);
  robot::leftMtrs.move(0);
  robot::rightMtrs.move(0);

}
else if (autonSelect == 17){
  //BlueGoalSafe
  pros::delay(1000);
  pros::Task lbTask(_1028A::auton::lbTask);
  robot::LBS.set_position(100);
  armtarget = 520;
  robot::chassis.moveToPoint(0, 3, 400, {}, false);
  pros::delay(300);
  pros::Task intakeTask(intakeControlTask);
  setIntakeState(IntakeState::COLOR_SORT_RED);
  robot::chassis.moveToPose(-24, -29, 74, 3000, {.forwards = false, .minSpeed = 60}, true);
  pros::delay(100);
  reset = 1;
  robot::chassis.waitUntil(50);
  robot::mogo.set_value(1);
  pros::delay(200);
  robot::chassis.moveToPoint(1, -45, 1000, {.maxSpeed = 80, .earlyExitRange = 12}, true);
  armtarget = 610;
  pros::delay(200);
  robot::chassis.waitUntilDone();
  robot::chassis.moveToPoint(30.8, -28, 2000, {.maxSpeed = 90, .minSpeed = 60}, false);
  robot::leftMtrs.move(127);
  robot::rightMtrs.move(127);
  pros::delay(500);
  robot::leftMtrs.move(0);
  robot::rightMtrs.move(0);
  robot::chassis.turnToPoint(-36, -30, 1000, {}, true);
  pros::delay(900);
  setIntakeState(IntakeState::STOP);
  armtarget = 0; 
  // robot::chassis.moveToPose(-43.75, -22, -95,  5000, {.maxSpeed = 60, .minSpeed = 40}, true);
  // robot::chassis.waitUntil(40);
  // robot::stickL.set_value(1);
  // robot::chassis.waitUntilDone();
  // robot::chassis.moveToPose(-9.5, -22.7, -95, 2500, {.forwards = false, .minSpeed = 70}, false);
  // robot::stickL.set_value(0);
  // setIntakeState(IntakeState::COLOR_SORT_BLUE);
  pros::delay(500);
  robot::chassis.moveToPoint(-26, -29, 1000, {}, true);
  lbTask.suspend();

}
else if (autonSelect == 19){
  //GoalScoreRushRedModd39K
  robot::chassis.moveToPose(5.2, 38.5, 23, 1000, {.minSpeed = 100}, true);
  robot::chassis.waitUntil(25);
  pros::Task lbTask(_1028A::auton::lbTask);
  armtarget = 750;
  robot::chassis.turnToHeading(34.5, 400, {}, false);
  pros::delay(200);
  robot::chassis.moveToPoint(-14, 30, 1000, {.forwards = false, .minSpeed = 70}, true);
  robot::chassis.waitUntil(30);
  robot::mogo.set_value(1);
  pros::Task checkINTAke(checkIntake);
  pros::delay(200);
  checkINTAke.suspend();
  pros::Task intakeTask(intakeControlTask);
  setIntakeState(IntakeState::COLOR_SORT_BLUE);
  robot::chassis.moveToPose(13, 32, 80, 1500, {.minSpeed = 70}, false);
  robot::chassis.turnToPoint(5, 23, 800, {}, false);
  reset = 1;
  robot::chassis.moveToPoint(5, 0, 1000, {.earlyExitRange = 8}, false);
  robot::chassis.moveToPose(53, -14, 90, 1400, {.minSpeed = 60}, true);
  robot::chassis.waitUntil(20);
  intakeTask.suspend();
  armtarget = 610;
  pros::Task que(queueDisk);
  robot::chassis.waitUntilDone();
  robot::leftMtrs.move(127);
  robot::rightMtrs.move(127);
  pros::delay(600);
  robot::leftMtrs.move(0);
  robot::rightMtrs.move(0);

  robot::leftMtrs.move(-80);
  robot::rightMtrs.move(-80);
  pros::delay(400);
  robot::leftMtrs.move(0);
  robot::rightMtrs.move(0);
  pros::delay(300);
  //robot::chassis.moveToPoint(24, -8, 800, {.forwards =false, .minSpeed = 60}, false);
  robot::stickR.set_value(1);
  robot::leftMtrs.move(50);
  robot::rightMtrs.move(50);
  pros::delay(550);
  robot::leftMtrs.move(0);
  robot::rightMtrs.move(0);
  robot::chassis.turnToHeading(-30, 1000, {}, true);
  pros::delay(100);
  armtarget = 105;
  robot::chassis.waitUntilDone();
  // robot::chassis.moveToPose(59, 30,42, 1300, {.lead = 0.65, .minSpeed = 70}, true);
  intakeTask.resume();
  setIntakeState(IntakeState::COLOR_SORT_BLUE);
  
  robot::chassis.moveToPose(57, 34,45, 2300, {.lead = 0.65, .minSpeed = 70}, true);
  pros::delay(400);
  robot::stickR.set_value(0);
  pros::delay(100);
  robot::chassis.waitUntilDone();
  intakeTask.suspend();
  robot::intake.move(-127);
  pros::delay(20);
  robot::intake.move(0);
  robot::chassis.turnToHeading(45, 400, {}, true);
  armtarget = 450;
  pros::delay(2500);
  lbTask.suspend();
  robot::LB.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);

  
}
else if (autonSelect == 100){
  //skills
  pros::Task odo(odomRead);
  pros::Task checkintake{checkIntake};
  robot::chassis.setPose(0,-3,0);
  pros::Task lbTask(_1028A::auton::lbTask);
  robot::LBS.set_position(100);
  armtarget = 500;
  pros::delay(500);
  reset = 1;
  robot::chassis.moveToPose(30, -5, -108, 1800, {.forwards = false, .minSpeed = 70}, false);
  robot::mogo.set_value(1);
  pros::delay(100);
  robot::chassis.moveToPoint(37, -38, 2000, {.minSpeed = 70}, true);
  robot::intake.move(127);
  robot::chassis.waitUntilDone();
  robot::chassis.moveToPose(54, -74, -200, 1700, {.minSpeed = 70}, false);
  robot::chassis.moveToPoint(59, -89, 2000, {.minSpeed = 40}, true);
  pros::delay(500);
  armtarget = 120;
  checkintake.suspend();
  robot::chassis.waitUntilDone();
  pros::delay(300);
  robot::chassis.moveToPose(47, -46.9, -196, 2200, {.forwards = false, .minSpeed = 60}, false);
  pros::delay(200);
  robot::chassis.turnToHeading(-265, 800, {}, false);
  robot::intake.move(-127);
  pros::delay(20);
  robot::intake.move(0);
  armtarget = 200;
  pros::delay(200);
  pros::Task queue(queueDisk);
  legacy::forward(17.5, -265, 127, 1100, 1);
  armtarget = 610;
  pros::delay(600);
  armtarget = 120;
  pros::delay(400);
  robot::intake.move(127);
  pros::delay(600);
  robot::intake.move(-127);
  pros::delay(20);
  robot::intake.move(0);
  armtarget = 610;
  pros::delay(600);
  legacy::forward(-12.5, -265, 127, 950, 1);
  robot::chassis.turnToHeading(-353, 500, {}, false);
  robot::intake.move(127);
  checkintake.resume();
  armtarget = 0;
  robot::chassis.moveToPose(57, 9, -354, 2500, {.maxSpeed = 80, .minSpeed = 50}, false);
  pros::delay(400);
  robot::chassis.turnToHeading(90, 800, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .earlyExitRange = 3}, false);
  robot::chassis.moveToPose(67, -15, -194, 2000, {.minSpeed = 50}, false);
  pros::delay(200);
  robot::chassis.turnToHeading(-155, 500, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE}, false);
  robot::chassis.moveToPoint(73, 8, 1000, {.forwards = false, .minSpeed = 50}, false);
  pros::delay(500);
  robot::mogo.set_value(0);
  robot::intake.move(-60);
  robot::chassis.moveToPoint(65, -3, 1000, {.earlyExitRange = 1}, false);



  
  robot::chassis.turnToHeading(-267, 1000, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE}, false);
  robot::chassis.moveToPose(-24, 0, -269, 4000, {.forwards = false, .maxSpeed = 100, .minSpeed = 40, . earlyExitRange = 1}, false);
  robot::mogo.set_value(1);
  pros::delay(100);
  robot::intake.move(127);
  robot::chassis.moveToPoint(-8, 0, 3500, {.minSpeed = 50, .earlyExitRange = 1}, false);
  robot::chassis.moveToPoint(-32, -38, 2000, {.minSpeed = 70, .earlyExitRange = 3}, true);
  robot::chassis.moveToPoint(-40, -62, 1700, {.maxSpeed = 75, .minSpeed = 40, .earlyExitRange = 3}, false);
  robot::chassis.moveToPoint(-52, -89, 2000, {.maxSpeed = 75, .minSpeed = 40, .earlyExitRange = 4}, true);
  pros::delay(800);
  armtarget = 120;
  checkintake.suspend();
  robot::chassis.waitUntilDone();
  robot::chassis.moveToPose(-34, -44, -154, 2000, {.forwards = false, .minSpeed = 60}, false);
  pros::delay(200);
  robot::chassis.turnToHeading(-90, 1000, {}, false);
  robot::intake.move(-127);
  pros::delay(20);
  robot::intake.move(0);
  armtarget = 200;
  pros::delay(200);
  pros::Task quEue(queueDisk);
  legacy::forward(17.5, -90, 127, 1100, 1);
  armtarget = 610;
  pros::delay(600);
  armtarget = 120;
  pros::delay(400);
  robot::intake.move(127);
  pros::delay(600);
  robot::intake.move(-127);
  pros::delay(20);
  robot::intake.move(0);
  armtarget = 610;
  pros::delay(600);
  legacy::forward(-12.5, -90, 127, 950, 1);
  pros::delay(100);
  robot::chassis.turnToHeading(0, 500, {}, false);
  robot::intake.move(127);
  checkintake.resume();
  robot::chassis.moveToPoint(-41, 8, 2500, {.maxSpeed = 75}, false);
  robot::chassis.turnToHeading(-110, 700, {}, false);
  robot::chassis.moveToPose(-48, -20, -188, 1500, {.minSpeed = 60}, false);
  robot::mogo.set_value(0);
  robot::chassis.moveToPoint(-51, 11, 1000, {.forwards = false, .minSpeed = 60, .earlyExitRange = 1.5}, false);
  robot::intake.move(-20);



  /*
  //checkintake.suspend();
  robot::chassis.moveToPose(-17, -24, -135, 2000, {.minSpeed = 40}, false);
  checkintake.suspend();
  robot::chassis.moveToPose(-40, -69, -164, 2000, {.minSpeed = 70, .earlyExitRange = 2}, false);
  pros::delay(900);
  armtarget = 120;
  robot::chassis.moveToPose(-47.5, -92, -160.5, 2000, {.minSpeed = 70, .earlyExitRange = 2}, false);
  pros::delay(200);
  robot::chassis.moveToPose(-32, -47.5, -154, 2000, {.forwards = false}, false);
  robot::chassis.turnToHeading(-90, 1000, {}, false);
  robot::intake.move(-127);
  pros::delay(20);
  robot::intake.move(0);
  armtarget = 200;
  pros::delay(200);
  pros::Task Queue(queueDisk);
  robot::chassis.moveToPoint(robot::chassis.getPose().x - 20, robot::chassis.getPose().y, 1800, {.maxSpeed = 50, .earlyExitRange = 2}, false);
  armtarget = 580;
  pros::delay(900);
  armtarget = 120;
  pros::delay(800);
  robot::intake.move(127);
  pros::delay(600);
  robot::intake.move(-127);
  pros::delay(20);
  robot::intake.move(0);
  armtarget = 580;
  pros::delay(900);
  robot::chassis.moveToPoint(robot::chassis.getPose().x + 10, robot::chassis.getPose().y, 1800, {.forwards = false, .earlyExitRange = 2}, false);
  checkintake.resume();
  armtarget = 0;
  robot::chassis.turnToHeading(0, 500, {}, false);
  robot::intake.move(127);
  robot::chassis.moveToPoint(-41, 8, 2500, {.maxSpeed = 75}, false);
  robot::chassis.turnToHeading(-110, 700, {}, false);
  robot::chassis.moveToPose(-48, -20, -188, 1500, {.minSpeed = 60}, false);
  robot::chassis.moveToPoint(-57.9, 8, 1000, {.forwards = false, .minSpeed = 60, .earlyExitRange = 1.5}, false);
  robot::intake.move(-20);
  robot::mogo.set_value(0);
  checkintake.suspend();
  
  robot::chassis.turnToHeading(-220.5, 900, {}, false);
  robot::intake.move(0);
  robot::chassis.moveToPoint(35, -81, 2500, {.minSpeed = 50}, false);
  
  pros::delay(300);
  robot::chassis.moveToPose(0.5, -103, -306, 2000, {.forwards = false, .minSpeed = 50}, false);
  robot::intake.move(-60);
  robot::mogo.set_value(1);
  checkintake.resume();
  robot::intake.move(127);
  pros::delay(200);
  robot::chassis.moveToPoint(65, -90, 2000, {.minSpeed = 60}, false);
  robot::chassis.turnToHeading(0, 800, {.earlyExitRange = 4}, false);
  robot::mogo.set_value(0);
  pros::delay(150);
  robot::leftMtrs.move(-80);
  robot::rightMtrs.move(-80);
  pros::delay(900);
  robot::leftMtrs.move(0);
  robot::rightMtrs.move(0);
  robot::chassis.moveToPoint(robot::chassis.getPose().x, robot::chassis.getPose().y + 10, 1000, {.minSpeed = 50}, false);
  robot::chassis.turnToPoint(65, -88, 800, {.minSpeed = 70, .earlyExitRange = 5}, false);
  robot::chassis.moveToPoint(30,-88, 2000, {.forwards = false, .minSpeed = 75, .earlyExitRange = 8}, false);
  robot::chassis.moveToPose(-58, -117, 90, 3500, {.forwards = false, .minSpeed = 75}, false);
  robot::chassis.moveToPoint(robot::chassis.getPose().x + 10, robot::chassis.getPose().y + 10, 1000, {.minSpeed = 60, .earlyExitRange = 8}, false);
  robot::chassis.turnToHeading(-130, 500, {}, false);
  checkintake.suspend();
  robot::intake.move(0);
  robot::chassis.moveToPose(5, -52, -135, 2500, {.forwards = false, .minSpeed = 60}, true);
  robot::chassis.waitUntil(15);
  armtarget = 600;
 robot::chassis.waitUntilDone();
 robot::leftMtrs.move(127);
 robot::rightMtrs.move(127);
 pros::delay(100);
 robot::leftMtrs.move(0);
 robot::rightMtrs.move(0);
 */
}
else if (autonSelect == 101){
  //skills
  pros::Task odo(odomRead);
  pros::Task checkintake{checkIntake};
  robot::chassis.setPose(0,-3,0);
  pros::Task lbTask(_1028A::auton::lbTask);
  robot::LBS.set_position(100);
  armtarget = 500;
  pros::delay(500);
  reset = 1;
  robot::chassis.moveToPose(30, -5, -108, 1800, {.forwards = false, .minSpeed = 70}, false);
  robot::mogo.set_value(1);
  pros::delay(100);
  robot::chassis.moveToPoint(37, -37, 2000, {.minSpeed = 70}, true);
  robot::intake.move(127);
  robot::chassis.waitUntilDone();
  robot::chassis.moveToPose(54, -74, -200, 1700, {.minSpeed = 70}, true);
  pros::delay(700);
  armtarget = 120;
  checkintake.suspend();
  robot::chassis.waitUntilDone();
  ///armtarget = 120;
  //robot::chassis.moveToPoint(59, -89, 2000, {.minSpeed = 40}, false);
  //checkintake.suspend();
  //pros::delay(300);
  //robot::chassis.moveToPose(43, -50, -196, 2000, {.forwards = false, .minSpeed = 40}, false);
  robot::chassis.moveToPoint(52, -49.5, 2000, {.forwards = false ,.earlyExitRange = 3}, false);
  armtarget = 300;
  robot::intake.move(-127);
  pros::delay(20);
  robot::intake.move(0);
  robot::chassis.turnToHeading(-265, 1000, {}, false);
  pros::Task queue(queueDisk);
  //armtarget = 200;
  //pros::delay(200);
  //pros::Task queue(queueDisk);
  robot::chassis.moveToPoint(robot::chassis.getPose().x + 20, robot::chassis.getPose().y, 1000, {.maxSpeed = 50, .earlyExitRange = 2}, false);
  robot::intake.move(-127);
  pros::delay(20);
  robot::intake.move(0);
  armtarget = 580;
  pros::delay(200);
  robot::intake.move(127);
  pros::delay(500);
  // armtarget = 120;
  // pros::delay(600);
  // robot::intake.move(127);
  // pros::delay(600);
  // robot::intake.move(-127);
  // pros::delay(20);
  // robot::intake.move(0);
  // armtarget = 580;
  // pros::delay(600);
  robot::chassis.moveToPoint(robot::chassis.getPose().x - 8, robot::chassis.getPose().y, 1800, {.forwards = false, .earlyExitRange = 2}, false);
  robot::chassis.turnToHeading(-353, 500, {}, false);
  robot::intake.move(127);
  checkintake.resume();
  armtarget = 0;
  robot::chassis.moveToPose(64, 12, -354, 2500, {.maxSpeed = 80, .minSpeed = 50}, false);
  pros::delay(400);
  robot::chassis.turnToHeading(90, 800, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .earlyExitRange = 3}, false);
  robot::chassis.moveToPose(78, -15, -194, 2000, {.minSpeed = 50}, false);
  pros::delay(200);
  robot::chassis.turnToHeading(-155, 500, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE}, false);
  robot::chassis.moveToPoint(80, 8, 1000, {.forwards = false, .minSpeed = 50}, false);
  pros::delay(500);
  robot::mogo.set_value(0);
  robot::intake.move(-60);
  robot::chassis.moveToPoint(65, 0, 1000, {.earlyExitRange = 1}, false);
  robot::chassis.turnToHeading(-267, 1000, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE}, false);
  robot::chassis.moveToPose(-24, 4, -269, 4000, {.forwards = false, .maxSpeed = 100, .minSpeed = 40, . earlyExitRange = 1}, false);
  robot::mogo.set_value(1);
  pros::delay(100);
  robot::intake.move(127);
  robot::chassis.moveToPoint(-12, 5, 3500, {.minSpeed = 50, .earlyExitRange = 1}, false);
  robot::chassis.moveToPose(-17, -24, -135, 2000, {.minSpeed = 40}, false);
  //checkintake.suspend();
  robot::chassis.moveToPose(-40, -69, -164, 2000, {.minSpeed = 70, .earlyExitRange = 2}, false);
  checkintake.suspend();
  armtarget = 120;
  //pros::delay(900);
  //armtarget = 120;
  //robot::chassis.moveToPose(-50, -92, -160.5, 2000, {.minSpeed = 70, .earlyExitRange = 2}, false);
  pros::delay(200);
  robot::chassis.moveToPoint(-30, -45, 1500, {.forwards = false, .minSpeed = 40, .earlyExitRange = 2}, false);
  pros::delay(300);
  armtarget = 300;
  robot::intake.move(-127);
  pros::delay(20);
  robot::intake.move(0);
  robot::chassis.turnToHeading(-90, 1000, {}, false);
  pros::Task Queue(queueDisk);
  //armtarget = 200;
  //pros::delay(200);
  //pros::Task queue(queueDisk);
  robot::chassis.moveToPoint(robot::chassis.getPose().x - 20, robot::chassis.getPose().y, 1800, {.maxSpeed = 50, .earlyExitRange = 2}, false);
  robot::intake.move(-127);
  pros::delay(20);
  robot::intake.move(0);
  armtarget = 580;
  pros::delay(200);
  robot::intake.move(127);
  pros::delay(500);
  robot::chassis.moveToPoint(robot::chassis.getPose().x + 10, robot::chassis.getPose().y, 1800, {.forwards = false, .earlyExitRange = 2}, false);
  checkintake.resume();
  armtarget = 0;
  robot::chassis.turnToHeading(0, 500, {}, false);
  robot::intake.move(127);
  robot::chassis.moveToPoint(-35, 8, 2500, {.maxSpeed = 75}, false);
  robot::chassis.turnToHeading(-110, 700, {}, false);
  robot::chassis.moveToPose(-48, -20, -188, 1500, {.minSpeed = 60}, false);
  robot::chassis.moveToPoint(-52, 8, 1200, {.forwards = false, .minSpeed = 60, .earlyExitRange = 1.5}, false);
  robot::intake.move(-20);
  robot::mogo.set_value(0);
  checkintake.suspend();
  
  robot::chassis.turnToHeading(-216, 1200, {.minSpeed = 30}, false);
  robot::intake.move(0);
  robot::chassis.moveToPoint(35, -81, 2500, {.minSpeed = 50}, true);
  robot::chassis.waitUntil(100);
  armtarget = 120;
  robot::intake.move(127);
  pros::delay(300);
  robot::chassis.moveToPose(-0.5, -103, -306, 2000, {.forwards = false, .minSpeed = 50, .earlyExitRange = 4}, false);
  robot::mogo.set_value(1);
  pros::delay(200);
  robot::chassis.moveToPose(8, -100, -306, 2000, {.minSpeed = 50}, false);
  robot::chassis.turnToHeading(174, 900, {}, false);
  pros::delay(200);
  robot::chassis.moveToPoint(robot::chassis.getPose().x, robot::chassis.getPose().y - 18, 2000, {.minSpeed = 50}, false);
  robot::intake.move(-127);
  pros::delay(20);
  robot::intake.move(0);
  armtarget = 640;
  robot::leftMtrs.move(-50);
  robot::rightMtrs.move(-50);
  pros::delay(300);
  robot::leftMtrs.move(0);
  robot::rightMtrs.move(0);
  

 //jaja 
  /*
  robot::intake.move(127);
  pros::delay(200);
  robot::chassis.moveToPoint(65, -90, 2000, {.minSpeed = 60}, false);
  robot::chassis.turnToHeading(0, 800, {.earlyExitRange = 4}, false);
  robot::mogo.set_value(0);
  robot::leftMtrs.move(-80);
  robot::rightMtrs.move(-80);
  pros::delay(900);
  robot::leftMtrs.move(0);
  robot::rightMtrs.move(0);
  robot::chassis.moveToPoint(robot::chassis.getPose().x, robot::chassis.getPose().y + 10, 1000, {.minSpeed = 50}, false);
  robot::chassis.turnToPoint(65, -88, 800, {.minSpeed = 70, .earlyExitRange = 5}, false);
  robot::chassis.moveToPoint(30,-88, 2000, {.forwards = false, .minSpeed = 75, .earlyExitRange = 8}, false);
  robot::chassis.moveToPose(-58, -117, 90, 3500, {.forwards = false, .minSpeed = 75}, false);
  robot::chassis.moveToPoint(robot::chassis.getPose().x + 10, robot::chassis.getPose().y + 10, 1000, {.minSpeed = 60, .earlyExitRange = 8}, false);
  robot::chassis.turnToHeading(-130, 500, {}, false);
  checkintake.suspend();
  robot::intake.move(0);
  robot::chassis.moveToPose(5, -52, -135, 2500, {.forwards = false, .minSpeed = 60}, true);
  robot::chassis.waitUntil(15);
  armtarget = 600;
 robot::chassis.waitUntilDone();
 robot::leftMtrs.move(127);
 robot::rightMtrs.move(127);
 pros::delay(100);
 robot::leftMtrs.move(0);
 robot::rightMtrs.move(0);
 */
 
}
else if (autonSelect == 102){
  pros::Task odo(odomRead);
  pros::Task checkintake{checkIntake};
  robot::chassis.setPose(0,-3,0);
  pros::Task lbTask(_1028A::auton::lbTask);
  robot::LBS.set_position(100);
  armtarget = 500;
  pros::delay(500);
  reset = 1;
  robot::chassis.moveToPose(30, -5, -108, 1800, {.forwards = false, .minSpeed = 70}, false);
  robot::mogo.set_value(1);
  pros::delay(100);
  robot::chassis.moveToPoint(37, -37, 2000, {.minSpeed = 70}, true);
  robot::intake.move(127);
  robot::chassis.waitUntilDone();
  robot::chassis.moveToPose(54, -74, -200, 1700, {.minSpeed = 70}, false);
  robot::chassis.moveToPoint(59, -89, 2000, {.minSpeed = 40}, true);
  robot::chassis.waitUntil(60);
  armtarget = 120;
  robot::chassis.waitUntilDone();
  checkintake.suspend();
  pros::delay(300);
  robot::chassis.moveToPoint(45, -50, 1300, {.forwards = false, .maxSpeed = 80, .minSpeed = 30}, false);
  pros::delay(300);
  robot::chassis.turnToHeading(-265, 800, {}, false);
  robot::intake.move(-127);
  pros::delay(20);
  robot::intake.move(0);
  armtarget = 200;
  pros::delay(200);
  pros::Task queue(queueDisk);
  robot::chassis.moveToPoint(robot::chassis.getPose().x + 20, robot::chassis.getPose().y, 1800, {.maxSpeed = 50, .earlyExitRange = 2}, false);
  armtarget = 680;
  pros::delay(800);
  armtarget = 120;
  pros::delay(550);
  robot::intake.move(127);
  pros::delay(500);
  robot::intake.move(-127);
  pros::delay(20);
  robot::intake.move(0);
  armtarget = 630;
  pros::delay(500);
  robot::chassis.moveToPoint(robot::chassis.getPose().x - 8, robot::chassis.getPose().y, 1800, {.forwards = false, .earlyExitRange = 2}, false);
  robot::chassis.turnToHeading(-353, 500, {}, false);
  robot::intake.move(127);
  checkintake.resume();
  armtarget = 0;
  robot::chassis.moveToPose(58.5, 13, -354, 2500, {.maxSpeed = 80, .minSpeed = 50}, false);
  pros::delay(400);
  robot::intake.move(127);
  robot::chassis.turnToHeading(90, 800, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .earlyExitRange = 3}, false);
  robot::intake.move(127);
  robot::chassis.moveToPose(65, -15, -194, 2000, {.minSpeed = 50}, false);
  pros::delay(200);
  robot::chassis.turnToHeading(-155, 500, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE}, false);
  robot::chassis.moveToPoint(73, 8, 1000, {.forwards = false, .minSpeed = 50}, false);
  pros::delay(500);
  robot::intake.move(-60);
  robot::mogo.set_value(0);



  robot::chassis.moveToPoint(65, -1, 1000, {.earlyExitRange = 1}, false);
  robot::chassis.turnToHeading(-267, 1000, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE}, false);
  robot::chassis.moveToPose(-24, -0.5, -269, 4500, {.forwards = false, .maxSpeed = 100, .minSpeed = 40, . earlyExitRange = 1}, false);
  robot::mogo.set_value(1);
  pros::delay(100);
  robot::intake.move(127);
  robot::chassis.moveToPoint(-15, 0.8, 3500, {.minSpeed = 50, .earlyExitRange = 1}, false);
  //checkintake.suspend();
  robot::chassis.moveToPose(-17, -24, -135, 2000, {.minSpeed = 40}, false);
  checkintake.suspend();
  robot::chassis.moveToPose(-40, -69, -164, 2000, {.minSpeed = 70, .earlyExitRange = 2}, false);
  robot::chassis.moveToPose(-45, -90, -160.5, 2000, {.minSpeed = 70, .earlyExitRange = 2}, true);
  robot::chassis.waitUntil(90);
  armtarget = 120;
  pros::delay(200);
  robot::chassis.moveToPoint(-28, -52, 2000, {.forwards = false,.maxSpeed = 80, .minSpeed = 30}, false);
  robot::chassis.turnToHeading(-90, 1000, {}, false);
  robot::intake.move(-127);
  pros::delay(20);
  robot::intake.move(0);
  armtarget = 200;
  pros::delay(200);
  pros::Task Queue(queueDisk);
  robot::chassis.moveToPoint(robot::chassis.getPose().x - 20, robot::chassis.getPose().y, 1800, {.maxSpeed = 50, .earlyExitRange = 2}, false);
  armtarget = 680;
  pros::delay(900);
  armtarget = 120;
  pros::delay(550);
  robot::intake.move(127);
  pros::delay(400);
  robot::intake.move(-127);
  pros::delay(20);
  robot::intake.move(0);
  armtarget = 630;
  pros::delay(400);
  robot::chassis.moveToPoint(robot::chassis.getPose().x + 10, robot::chassis.getPose().y, 1800, {.forwards = false, .earlyExitRange = 2}, false);
  checkintake.resume();
  armtarget = 0;
  robot::chassis.turnToHeading(0, 500, {}, false);
  robot::intake.move(127);
  robot::chassis.moveToPoint(-41, 8, 2500, {.maxSpeed = 75}, false);
  robot::chassis.turnToHeading(-110, 700, {}, false);
  robot::chassis.moveToPose(-48, -20, -188, 1500, {.minSpeed = 60}, false);
  robot::chassis.moveToPoint(-57.9, 8, 1400, {.forwards = false, .minSpeed = 60, .earlyExitRange = 1.5}, false);
  robot::intake.move(-20);
  robot::mogo.set_value(0);
  checkintake.suspend();
  robot::chassis.turnToHeading(-218, 900, {.minSpeed = 40}, false);
  robot::intake.move(0);
  robot::chassis.moveToPoint(35, -81, 2500, {.minSpeed = 50}, true);
  robot::chassis.waitUntil(120);
  armtarget = 120;
  robot::intake.move(127);
  pros::delay(300);
  robot::chassis.moveToPose(-1, -102, -306, 2500, {.forwards = false, .minSpeed = 50}, false);
  robot::mogo.set_value(1);
  pros::delay(300);
  robot::chassis.moveToPoint(0, -97, 900, {.minSpeed = 50}, false);
  robot::chassis.turnToHeading(174, 900, {}, false);
  pros::delay(200);
  robot::chassis.moveToPoint(robot::chassis.getPose().x, robot::chassis.getPose().y - 25, 1000, {.minSpeed = 50}, false);
  robot::intake.move(-127);
  pros::delay(20);
  robot::intake.move(0);
  armtarget = 640;
  robot::leftMtrs.move(-50);
  robot::rightMtrs.move(-50);
  pros::delay(410);
  robot::leftMtrs.move(0);
  robot::rightMtrs.move(0);
  armtarget = 0;
  robot::chassis.turnToHeading(-90, 1000, {}, false);
  robot::chassis.moveToPoint(30, 92, 1500, {.minSpeed = 50}, false);


}
else if (autonSelect == 103){
  pros::Task odo(odomRead);
  pros::Task checkintake{checkIntake};
  robot::chassis.setPose(0,-3,0);
  pros::Task lbTask(_1028A::auton::lbTask);
  robot::LBS.set_position(100);
  armtarget = 500;
  pros::delay(500);
  reset = 1;
  robot::chassis.moveToPose(30, -5, -108, 1800, {.forwards = false, .minSpeed = 70}, false);
  robot::mogo.set_value(1);
  pros::delay(100);
  robot::chassis.moveToPoint(37, -37, 2000, {.minSpeed = 70}, true);
  robot::intake.move(127);
  robot::chassis.waitUntilDone();
  robot::chassis.moveToPose(54, -74, -200, 1700, {.minSpeed = 70}, false);
  robot::chassis.moveToPoint(59, -89, 2000, {.minSpeed = 40}, true);
  robot::chassis.waitUntil(100);
  armtarget = 120;
  robot::chassis.waitUntilDone(); 
  checkintake.suspend();
  pros::delay(300);
  robot::chassis.moveToPose(43, -48.5, -196, 2000, {.forwards = false, .minSpeed = 40}, false);
  robot::chassis.turnToHeading(-265, 1000, {}, false);
  robot::intake.move(-127);
  pros::delay(20);
  robot::intake.move(0);
  armtarget = 200;
  pros::delay(200);
  pros::Task queue(queueDisk);
  robot::chassis.moveToPoint(robot::chassis.getPose().x + 20, robot::chassis.getPose().y, 1800, {.maxSpeed = 50, .earlyExitRange = 2}, false);
  armtarget = 580;
  pros::delay(600);
  armtarget = 120;
  pros::delay(600);
  robot::intake.move(127);
  pros::delay(600);
  robot::intake.move(-127);
  pros::delay(20);
  robot::intake.move(0);
  armtarget = 580;
  pros::delay(600);
  robot::chassis.moveToPoint(robot::chassis.getPose().x - 10, robot::chassis.getPose().y, 1800, {.forwards = false, .earlyExitRange = 2}, false);
  robot::chassis.turnToHeading(-353, 500, {}, false);
  robot::intake.move(127);
  checkintake.resume();
  armtarget = 0;
  robot::chassis.moveToPose(57, 9, -354, 2500, {.maxSpeed = 80, .minSpeed = 50}, false);
  pros::delay(400);
  robot::chassis.turnToHeading(90, 800, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .earlyExitRange = 3}, false);
  robot::chassis.moveToPose(67, -15, -194, 2000, {.minSpeed = 50}, false);
  pros::delay(200);
  robot::chassis.turnToHeading(-155, 500, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE}, false);
  robot::chassis.moveToPoint(73, 8, 1000, {.forwards = false, .minSpeed = 50}, false);
  pros::delay(500);
  robot::mogo.set_value(0);
  robot::intake.move(-60);
  robot::chassis.moveToPoint(65, -3, 1000, {.earlyExitRange = 1}, false);
  robot::chassis.turnToHeading(-267, 1000, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE}, false);
  robot::chassis.moveToPose(-22, -1, -269, 3500, {.forwards = false, .maxSpeed = 100, .minSpeed = 40, . earlyExitRange = 1}, false);
  robot::mogo.set_value(1);
  pros::delay(100);
  robot::intake.move(127);
  robot::chassis.moveToPoint(-12, 0, 3500, {.minSpeed = 50, .earlyExitRange = 1}, false);
  //checkintake.suspend();
  robot::chassis.moveToPose(-17, -24, -135, 2000, {.minSpeed = 40}, false);
  checkintake.suspend();
  robot::chassis.moveToPose(-40, -69, -164, 2000, {.minSpeed = 70, .earlyExitRange = 2}, false);
  robot::chassis.moveToPose(-52, -92, -160.5, 2000, {.minSpeed = 70, .earlyExitRange = 2}, true);
  robot::chassis.waitUntil(100);
  armtarget = 120;
  robot::chassis.waitUntilDone();
  pros::delay(200);
  robot::chassis.moveToPose(-32, -47.5, -154, 2000, {.forwards = false}, false);
  robot::chassis.turnToHeading(-90, 1000, {}, false);
  robot::intake.move(-127);
  pros::delay(20);
  robot::intake.move(0);
  armtarget = 200;
  pros::delay(200);
  pros::Task Queue(queueDisk);
  robot::chassis.moveToPoint(robot::chassis.getPose().x - 20, robot::chassis.getPose().y, 1800, {.maxSpeed = 50, .earlyExitRange = 2}, false);
  armtarget = 580;
  pros::delay(900);
  armtarget = 120;
  pros::delay(800);
  robot::intake.move(127);
  pros::delay(600);
  robot::intake.move(-127);
  pros::delay(20);
  robot::intake.move(0);
  armtarget = 580;
  pros::delay(900);
  robot::chassis.moveToPoint(robot::chassis.getPose().x + 10, robot::chassis.getPose().y, 1800, {.forwards = false, .earlyExitRange = 2}, false);
  checkintake.resume();
  armtarget = 0;
  robot::chassis.turnToHeading(0, 500, {}, false);
  robot::intake.move(127);
  robot::chassis.moveToPoint(-41, 8, 2500, {.maxSpeed = 75}, false);
  robot::chassis.turnToHeading(-110, 700, {}, false);
  robot::chassis.moveToPose(-48, -20, -188, 1500, {.minSpeed = 60}, false);
  robot::chassis.moveToPoint(-57.9, 8, 1400, {.forwards = false, .minSpeed = 60, .earlyExitRange = 1.5}, false);
  robot::intake.move(-20);
  robot::mogo.set_value(0);
  checkintake.suspend();
  
  robot::chassis.turnToHeading(-219, 900, {}, false);
  robot::intake.move(0);
  robot::chassis.moveToPoint(35, -81, 2500, {.minSpeed = 50}, false);
  
  pros::delay(300);
  robot::chassis.moveToPose(0.5, -103, -306, 2000, {.forwards = false, .minSpeed = 50}, false);
  robot::intake.move(-60);
  robot::mogo.set_value(1);
  checkintake.resume();
  robot::intake.move(127);
  pros::delay(200);
  robot::chassis.moveToPoint(65, -94, 2000, {.minSpeed = 60}, false);
//   robot::chassis.turnToHeading(0, 800, {.earlyExitRange = 4}, false);
//   robot::mogo.set_value(0);
//   pros::delay(150);
//   robot::leftMtrs.move(-80);
//   robot::rightMtrs.move(-80);
//   pros::delay(900);
//   robot::leftMtrs.move(0);
//   robot::rightMtrs.move(0);
//   robot::chassis.moveToPoint(robot::chassis.getPose().x, robot::chassis.getPose().y + 10, 1000, {.minSpeed = 50}, false);
//   robot::chassis.turnToPoint(65, -88, 800, {.minSpeed = 70, .earlyExitRange = 5}, false);
//   robot::chassis.moveToPoint(30,-88, 2000, {.forwards = false, .minSpeed = 75, .earlyExitRange = 8}, false);
//   robot::chassis.moveToPose(-58, -117, 90, 3500, {.forwards = false, .minSpeed = 75}, false);
//   robot::chassis.moveToPoint(robot::chassis.getPose().x + 10, robot::chassis.getPose().y + 10, 1000, {.minSpeed = 60, .earlyExitRange = 8}, false);
//   robot::chassis.turnToHeading(-130, 500, {}, false);
//   checkintake.suspend();
//   robot::intake.move(0);
//   robot::chassis.moveToPose(5, -52, -135, 2500, {.forwards = false, .minSpeed = 60}, true);
//   robot::chassis.waitUntil(15);
//   armtarget = 600;
//  robot::chassis.waitUntilDone();
//  robot::leftMtrs.move(127);
//  robot::rightMtrs.move(127);
//  pros::delay(100);
//  robot::leftMtrs.move(0);
//  robot::rightMtrs.move(0);
// }
}
}
