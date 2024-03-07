#include "1028A/comp/driver.h"
#include "1028A/legacy.h"
#include "1028A/misc.h"
#include "1028A/robot.h"
#include "1028A/vars.h"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"

void _1028A::comp::driver::driveCTRL() {
  if (autonSelect == 12) {
    legacy::slantR(-40, 800);
    legacy::forward(-127, 500);
    legacy::forward(200, 127, 1, 600, 0, 0);
    legacy::turn(-170, 127, 1, 800, 0, 0);
    robot::flapR.set_value(1);
    Rwing = open;
    robot::kicker.move(-100);
    kickeron = 1;
    while (1) {
      if (_1028A::robot::master.get_analog(ANALOG_LEFT_Y) != 0 or
          _1028A::robot::master.get_analog(ANALOG_RIGHT_X) != 0) {
        break;
      }
      pros::delay(20);
    }
  }

  robot::leftfront.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  robot::leftmid.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  robot::leftback.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  robot::rightfront.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  robot::rightmid.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  robot::rightback.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

  while (1) {
    int power = _1028A::robot::master.get_analog(ANALOG_LEFT_Y);
    int turn = _1028A::robot::master.get_analog(ANALOG_RIGHT_X);

    _1028A::robot::leftMtrs.move(power + turn);
    _1028A::robot::rightMtrs.move(power - turn);

    if (_1028A::robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
      robot::kicker.move(-105);
    } else if (kickeron == 1) {
      robot::kicker.move(-105);
    } else {
      robot::kicker.move(0);
    }

    if (stickon == 1) {
      robot::stick.set_value(1);
    } else {
      robot::stick.set_value(0);
    }
    pros::delay(5);
  }
}

void _1028A::comp::driver::kickerCTRL() {
  while (1) {
    if (_1028A::robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) &&
        kickeron == 0) {
      kickeron = 1;
      pros::delay(400);
    } else if (_1028A::robot::master.get_digital(
                   pros::E_CONTROLLER_DIGITAL_R1) &&
               kickeron == 1) {
      kickeron = 0;
      pros::delay(400);
    }

    if (_1028A::robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT) &&
        stickon == 0) {
      stickon = 1;
      pros::delay(300);
    } else if (_1028A::robot::master.get_digital(
                   pros::E_CONTROLLER_DIGITAL_RIGHT) &&
               stickon == 1) {
      stickon = 0;
      pros::delay(300);
    }
    pros::delay(20);
  }
}

void _1028A::comp::driver::flapCTRL() {
  while (1) {
    if (robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
      if (Rwing == closed) {
        robot::flapR.set_value(1);
        Rwing = open;
        pros::delay(200);
      } else if (Rwing == open) {
        robot::flapR.set_value(0);
        Rwing = closed;
        pros::delay(200);
      }
    }

    if (robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
      if (Lwing == closed) {
        robot::flapL.set_value(1);
        Lwing = open;
        pros::delay(200);
      } else if (Lwing == open) {
        robot::flapL.set_value(0);
        Lwing = closed;
        pros::delay(200);
      }
    }

    if (robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
      if (Lwing != Rwing) {
        robot::flapL.set_value(1);
        robot::flapR.set_value(1);
        Lwing = open;
        Rwing = open;
        pros::delay(200);
      } else if (Lwing == Rwing && Lwing == closed) {
        robot::flapL.set_value(1);
        robot::flapR.set_value(1);
        Lwing = open;
        Rwing = open;
        pros::delay(200);
      } else if (Lwing == Rwing && Lwing == open) {
        robot::flapL.set_value(0);
        robot::flapR.set_value(0);
        Lwing = closed;
        Rwing = closed;
        pros::delay(200);
      } else {
      }
    }
    pros::delay(20);
  }
}

void _1028A::comp::driver::intakeCTRL() {
  while (1) {
    if (_1028A::robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
      _1028A::robot::intake.move(127);

    } else if (_1028A::robot::master.get_digital(
                   pros::E_CONTROLLER_DIGITAL_L2)) {
      _1028A::robot::intake.move(-127);
    } else {
      _1028A::robot::intake.move(0);
    }
    pros::delay(10);
  }
}

void _1028A::comp::driver::climbCTRL() {
  while (1) {
    if (_1028A::robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_R2) &&
        climb == neutral) {
      _1028A::robot::climb_set1.set_value(1);
      _1028A::robot::climb_set2.set_value(0);
      climb = up;
      pros::delay(300);
    } else if (_1028A::robot::master.get_digital(
                   pros::E_CONTROLLER_DIGITAL_R2) &&
               climb == up) {
      _1028A::robot::climb_set1.set_value(0);
      _1028A::robot::climb_set2.set_value(1);
      climb = down;
      pros::delay(300);
    } else if (_1028A::robot::master.get_digital(
                   pros::E_CONTROLLER_DIGITAL_R2) &&
               climb == down) {
      _1028A::robot::climb_set1.set_value(0);
      _1028A::robot::climb_set2.set_value(0);
      climb = neutral;
      pros::delay(300);
    }
    pros::delay(20);
  }
}

void _1028A::comp::driver::dataCTRL() {
  while (1) {
    if (overTemp == true) {
      _1028A::robot::master.print(0, 0, "Motors OverTemp");
    } else {
      if (Rwing == Lwing && Rwing == open) {
        _1028A::robot::master.print(0, 0, "Wing: Both");
      } else if (Lwing == open) {
        _1028A::robot::master.print(0, 0, "Wing: Left");
      } else if (Rwing == open) {
        _1028A::robot::master.print(0, 0, "Wing: Right");
      } else if (Lwing == closed && Rwing == closed) {
        _1028A::robot::master.print(0, 0, "Wing: None");
      }

      if (climb == up) {
        _1028A::robot::master.print(0, 10, "Climb: Up");
      } else if (climb == down) {
        _1028A::robot::master.print(0, 10, "Climb: Down");
      } else if (climb == neutral) {
        _1028A::robot::master.print(0, 10, "Climb: Neut");
      }
    }

    pros::delay(200);
  }
}