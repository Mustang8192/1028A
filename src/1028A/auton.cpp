#include "1028A/auton.h"
#include "1028A/robot.h"
#include "lemlib/chassis/chassis.hpp"
#include "pros/rtos.hpp"
#include "1028A/logger.h"
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
      while (1){
        if (_1028A::robot::LBSLimit.get_value()){
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

    if (!pros::competition::is_autonomous() && !pros::competition::is_disabled() && pros::competition::is_connected()){
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
    if (_1028A::robot::optical.get_proximity() > 253){
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
constexpr double BLUE_HUE_MIN = 80.0;
constexpr double BLUE_HUE_MAX = 300.0;
constexpr double RED_HUE_MIN  =   0.0;
constexpr double RED_HUE_MAX  =  70.0;
constexpr uint32_t DISC_TIMEOUT_MS     = 400;


void setIntakeState(IntakeState newState) {
  g_currentState = newState;
}


static DiskColor getDiskColor() {
  double hue = _1028A::robot::optical.get_hue();

  if (hue >= BLUE_HUE_MIN && hue <= BLUE_HUE_MAX) {
    return DiskColor::BLUE;
  }
  else if ((hue >= RED_HUE_MIN && hue <= RED_HUE_MAX)
        || (hue >= 330.0 && hue <= 360.0)) {
    // Red often appears near 0 or 360
    return DiskColor::RED;
  }
  return DiskColor::UNKNOWN;
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

  while (true) {
    if (!pros::competition::is_autonomous() && !pros::competition::is_disabled()) {
      break;
    }

    bool seesDisc = (_1028A::robot::optical.get_proximity() > 200); 

    if (seesDisc && !g_lastSawDisc) {
      DiskColor c = getDiskColor();
      if (c == DiskColor::BLUE || c == DiskColor::RED) {
        DiskInfo newDisk{c, pros::millis()};
        g_diskQueue.push(newDisk);

        std::cout << "[DEBUG] ** New Disc Detected **  Color=" 
                  << colorToString(c) 
                  << "  entryTime=" << newDisk.entryTime << "\n";
      } else {
        std::cout << "[DEBUG] Detected UNKNOWN color disc; not adding.\n";
      }
    }
    g_lastSawDisc = seesDisc;

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
      double topDist = _1028A::robot::distance.get();
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

        if (frontInfo.color == colorToEject) {
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
    pros::delay(20);
  }
}

void _1028A::auton::auton(){
  autonSelect = 100;
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
  robot::stick.set_value(1);
  robot::intake.move(127);
  pros::Task queued(queueDisk);
    robot::chassis.moveToPoint(0, 34, 2000, {.minSpeed=80}, false);
    robot::stick.set_value(0);
    robot::chassis.moveToPoint(0, 15, 2000, {.forwards = false}, true);
    robot::chassis.waitUntil(9);
    robot::stick.set_value(1);
    robot::chassis.turnToHeading(-260, 1000, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .earlyExitRange = 4}, false);
    robot::stick.set_value(0);
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
  robot::stick.set_value(1);
  robot::intake.move(127);
  pros::Task queued(queueDisk);
    robot::chassis.moveToPoint(0, 34, 2000, {.minSpeed=80}, false);
    robot::stick.set_value(0);
    robot::chassis.moveToPoint(0, 15, 2000, {.forwards = false}, true);
    robot::chassis.waitUntil(9);
    robot::stick.set_value(1);
    robot::chassis.turnToHeading(260, 1000, {.direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .earlyExitRange = 4}, false);
    robot::stick.set_value(0);
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
  robot::chassis.moveToPose(29, -27, -100, 1400, {.forwards = false, .minSpeed = 70, .earlyExitRange = 2}, true);
  robot::chassis.waitUntil(43);
  robot::mogo.set_value(1);
  reset = 1;
  robot::chassis.waitUntilDone();
  pros::delay(200);
  robot::chassis.turnToHeading(-197, 500, {.earlyExitRange = 6}, false);
  robot::intake.move(127);
  pros::Task checkintake(checkIntake);
  robot::chassis.turnToHeading(-182.5, 500, {}, false);
  robot::chassis.moveToPose(6, -54, -132, 1500, {.minSpeed = 60, .earlyExitRange = 6}, false);
  robot::chassis.turnToHeading(-150, 800, {}, false);
  robot::chassis.moveToPoint(16, -27, 1500, {.forwards = false, .minSpeed = 60}, false);
  robot::chassis.moveToPose(0, -37, -115, 1400, {.minSpeed = 80}, false);
  robot::chassis.moveToPoint(robot::chassis.getPose().x + 4, robot::chassis.getPose().y, 700, {.minSpeed = 50}, false);
  lbTask.suspend();
  robot::chassis.turnToPoint(0,0,1000, {.minSpeed = 40}, false);
  robot::intake.move(0);
  robot::chassis.moveToPoint(3, 0, 1500, {}, false);
  robot::chassis.turnToHeading(-90, 500, {}, false);
  //robot::chassis.turnToHeading(70, 500, {}, false);
  /*
  pros::delay(200);
  robot::chassis.moveToPose(-34, -25, 140, 2300, {.minSpeed = 50}, false);
  robot::leftMtrs.move(115);
  robot::rightMtrs.move(115);
  pros::delay(300);
  robot::leftMtrs.move(0);
  robot::rightMtrs.move(0);
  pros::delay(300);
  robot::chassis.turnToHeading(50, 800, {}, false);
  checkintake.suspend();
  */
  // robot::chassis.moveToPose(27, -15, 70, 3000, {}, true);
  // pros::delay(1300);
  // robot::chassis.waitUntilDone();
  /*
  pros::Task checkintake(checkIntake);
  robot::chassis.turnToHeading(-182.5, 500, {}, false);
  robot::chassis.moveToPose(8, -60, -132, 1500, {.minSpeed = 60, .earlyExitRange = 4}, false);
  
  robot::chassis.moveToPoint(8, -60, 1500, {.minSpeed = 60}, false);
  pros::delay(500);
  robot::chassis.turnToHeading(-150, 800, {}, false);
  robot::chassis.moveToPoint(26, -20, 1500, {.forwards = false, .minSpeed = 60}, false);
  robot::chassis.moveToPose(0, -37, -115, 1400, {.minSpeed = 80}, false);
  robot::chassis.moveToPoint(robot::chassis.getPose().x + 4, robot::chassis.getPose().y, 700, {.minSpeed = 50}, false);
  robot::chassis.turnToHeading(-90, 500, {}, false);
  pros::delay(200);
  robot::chassis.moveToPose(-31, -28, 133, 2300, {.minSpeed = 50}, false);
  robot::leftMtrs.move(92);
  robot::rightMtrs.move(92);
  pros::delay(300);
  robot::leftMtrs.move(0);
  robot::rightMtrs.move(0);
  pros::delay(300);
  robot::chassis.turnToHeading(50, 800, {}, false);
  checkintake.suspend();
  armtarget = 500;
  robot::chassis.moveToPose(30, -5, 47.5, 3000, {}, true);
  pros::delay(1300);
  lbTask.suspend();
  robot::chassis.waitUntilDone();
  */

}

else if (autonSelect == 6){
  //red5Ring
  /*
  pros::Task lbTask(_1028A::auton::lbTask);
  pros::Task odo(odomRead);
  armtarget = 500;
  robot::chassis.moveToPoint(0,1.5, 2000, {.minSpeed = 40}, false);
  pros::delay(200);
  robot::chassis.moveToPose(-32, -23, 90, 2000, {.forwards = false, .minSpeed = 70, .earlyExitRange = 2}, true);
  robot::chassis.waitUntil(43);
  robot::mogo.set_value(1);
  reset = 1;
  robot::chassis.waitUntilDone();
  pros::delay(200);
  robot::chassis.turnToHeading(164, 600, {.earlyExitRange = 3}, false);
  robot::intake.move(127);
  pros::Task checkintake(checkIntake);
  robot::chassis.moveToPose(-9, -60, 142, 1500, {.minSpeed = 60}, false);
  pros::delay(500);
  robot::chassis.turnToHeading(150, 800, {}, false);
  robot::chassis.moveToPoint(-24, -24, 1500, {.forwards = false, .minSpeed = 60}, false);
  robot::chassis.moveToPose(0, -37, 115, 1400, {.minSpeed = 80}, false);
  robot::chassis.moveToPoint(robot::chassis.getPose().x - 4, robot::chassis.getPose().y, 700, {.minSpeed = 50}, false);
  robot::chassis.turnToHeading(100, 500, {}, false);
  pros::delay(200);
  robot::chassis.moveToPose(30, -28, -133, 2300, {.minSpeed = 50}, false);
  robot::leftMtrs.move(70);
  robot::rightMtrs.move(70);
  pros::delay(300);
  robot::leftMtrs.move(0);
  robot::rightMtrs.move(0);
  pros::delay(300);
  robot::chassis.turnToHeading(-50, 800, {}, false);
  checkintake.suspend();
  armtarget = 500;
  robot::chassis.moveToPose(-32, 2, 47.5, 3000, {}, true);
  pros::delay(1300);
  lbTask.suspend();
  robot::chassis.waitUntilDone();
  */

 pros::Task lbTask(_1028A::auton::lbTask);
  pros::Task odo(odomRead);
  armtarget = 500;
  robot::chassis.moveToPoint(0,1.5, 2000, {.minSpeed = 55}, false);
  pros::delay(200);
  robot::chassis.moveToPose(-29, -27, 90, 1400, {.forwards = false, .minSpeed = 80, .earlyExitRange = 2}, true);
  robot::chassis.waitUntil(43);
  robot::mogo.set_value(1);
  reset = 1;
  robot::chassis.waitUntilDone();
  pros::delay(200);
  robot::chassis.turnToHeading(197, 400, {.earlyExitRange = 6}, false);
  robot::intake.move(127);
  pros::Task checkintake(checkIntake);
  robot::chassis.turnToHeading(182.5, 500, {}, false);
  robot::chassis.moveToPose(-6, -53, 132, 1500, {.minSpeed = 60, .earlyExitRange = 6}, false);
  robot::chassis.turnToHeading(150, 800, {}, false);
  robot::chassis.moveToPoint(-16, -24, 1500, {.forwards = false, .minSpeed = 60}, false);
  robot::chassis.moveToPose(7.2, -37.7, 107, 2000, {.minSpeed = 55}, false);
  robot::chassis.moveToPoint(robot::chassis.getPose().x - 4, robot::chassis.getPose().y, 700, {.minSpeed = 50}, false);
  lbTask.suspend();
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

  robot::chassis.turnToPoint(0,0,1000, {.minSpeed = 40}, false);
  robot::intake.move(0);
  robot::chassis.moveToPoint(3, 0, 1500, {}, false);
  robot::chassis.turnToHeading(-90, 500, {}, false);

}
else if (autonSelect == 7){
   //redRingRush5
  robot::chassis.setPose(0,0,0);
  robot::chassis.moveToPoint(-13, 40, 2000, {.earlyExitRange = 3}, true);
  pros::Task lbTask(_1028A::auton::lbTask);
   robot::LBS.set_position(100);
   armtarget = 180;
  robot::chassis.waitUntil(30);
  pros::Task disk(queueDisk);
  robot::chassis.waitUntilDone();
  pros::delay(300);
  robot::chassis.moveToPoint(10, 29, 2000, {.forwards = false, .earlyExitRange = 1.5}, true);
  robot::chassis.waitUntil(23);
  robot::mogo.set_value(1);
  robot::chassis.waitUntilDone();
  pros::Task intakeTask(intakeControlTask);
  setIntakeState(IntakeState::COLOR_SORT_BLUE);
  robot::chassis.moveToPoint(16.5, -5, 2000, {}, false);
  robot::chassis.turnToHeading(-227, 500, {}, false);
  armtarget = 500;
  pros::delay(800);
  robot::chassis.moveToPoint(0, 18, 1000, {.forwards = false}, false);
  robot::chassis.turnToPoint(-27, -4, 1000, {}, false);
  robot::chassis.moveToPoint(-27, -4, 1000, {}, true);
  robot::leftMtrs.move(127);
  robot::rightMtrs.move(127);
  pros::delay(300);
  robot::leftMtrs.move(0);
  robot::rightMtrs.move(0);
  pros::delay(500);
  robot::chassis.moveToPoint(-11, 3, 1000, {.forwards = false}, false);
  reset = 1;
  robot::chassis.turnToPoint(-16, 28, 500, {}, false);
  robot::chassis.moveToPoint(-20, 28, 2000, {.minSpeed = 40}, false);
  robot::chassis.moveToPoint(-20, 38.5, 2000, {.minSpeed = 40}, false);
  
  
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
  robot::chassis.moveToPoint(37, -37, 2000, {.minSpeed = 70}, true);
  robot::intake.move(127);
  robot::chassis.waitUntilDone();
  robot::chassis.moveToPose(54, -74, -200, 1700, {.minSpeed = 70}, false);
  pros::delay(900);
  armtarget = 98;
  robot::chassis.moveToPoint(59, -89, 2000, {.minSpeed = 40}, false);
  checkintake.suspend();
  pros::delay(300);
  robot::chassis.moveToPose(43, -50, -196, 2000, {.forwards = false, .minSpeed = 40}, false);
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
  armtarget = 98;
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
  reset = 1;
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
  pros::delay(900);
  armtarget = 98;
  robot::chassis.moveToPose(-50, -92, -160.5, 2000, {.minSpeed = 70, .earlyExitRange = 2}, false);
  pros::delay(200);
  robot::chassis.moveToPose(-32, -48.6, -154, 2000, {.forwards = false}, false);
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
  armtarget = 98;
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
  
  robot::chassis.turnToHeading(-221, 900, {}, false);
  robot::intake.move(0);
  robot::chassis.moveToPoint(35, -81, 2500, {.minSpeed = 50}, false);
  
  pros::delay(300);
  robot::chassis.moveToPose(0.5, -103, -306, 2000, {.forwards = false, .minSpeed = 50}, false);
  robot::intake.move(-60);
  robot::mogo.set_value(1);
  checkintake.resume();
  robot::intake.move(127);
  pros::delay(200);
  robot::chassis.moveToPoint(65, -88, 2000, {.minSpeed = 60}, false);
  robot::chassis.turnToHeading(0, 800, {.earlyExitRange = 4}, false);
  robot::mogo.set_value(0);
  robot::leftMtrs.move(-80);
  robot::rightMtrs.move(-80);
  pros::delay(900);
  robot::leftMtrs.move(0);
  robot::rightMtrs.move(0);
  robot::chassis.moveToPoint(robot::chassis.getPose().x, robot::chassis.getPose().y + 10, 1000, {.minSpeed = 50}, false);
  robot::chassis.turnToPoint(65, -88, 800, {.minSpeed = 70, .earlyExitRange = 5}, false);
  robot::chassis.moveToPoint(40,-88, 2000, {.forwards = false, .minSpeed = 75, .earlyExitRange = 8}, false);
  robot::chassis.moveToPose(-58, -117, 90, 3500, {.forwards = false, .minSpeed = 75}, false);
  robot::chassis.moveToPoint(robot::chassis.getPose().x + 10, robot::chassis.getPose().y + 10, 1000, {.minSpeed = 60, .earlyExitRange = 8}, false);
  robot::chassis.turnToHeading(-130, 500, {}, false);
  checkintake.suspend();
  robot::intake.move(0);
  robot::chassis.moveToPose(3, -59, -135, 2500, {.forwards = false, .minSpeed = 60}, true);
  robot::chassis.waitUntil(15);
  armtarget = 600;
 robot::chassis.waitUntilDone();
 robot::leftMtrs.move(127);
 robot::rightMtrs.move(127);
 pros::delay(100);
 robot::leftMtrs.move(0);
 robot::rightMtrs.move(0);

}
}
