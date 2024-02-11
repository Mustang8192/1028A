#include "1028A/comp/driver.h"
#include "1028A/misc.h"
#include "1028A/robot.h"
#include "1028A/vars.h"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"

void _1028A::comp::driver::driveCTRL() {
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
      robot::kicker.move(-127);
    } else if (kickeron == 1) {
      robot::kicker.move(-127);
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
      if (RwingSts == 0) {
        robot::flapR.set_value(1);
        RwingSts = 1;
        pros::delay(200);
      } else if (RwingSts == 1) {
        robot::flapR.set_value(0);
        RwingSts = 0;
        pros::delay(200);
      }
    }

    if (robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
      if (LwingSts == 0) {
        robot::flapL.set_value(1);
        LwingSts = 1;
        pros::delay(200);
      } else if (LwingSts == 1) {
        robot::flapL.set_value(0);
        LwingSts = 0;
        pros::delay(200);
      }
    }

    if (robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
      if (LwingSts != RwingSts) {
        robot::flapL.set_value(1);
        robot::flapR.set_value(1);
        LwingSts = 1;
        RwingSts = 1;
        BwingSts = 1;
        pros::delay(200);
      } else if (LwingSts == 0 && RwingSts == 0) {
        robot::flapL.set_value(1);
        robot::flapR.set_value(1);
        LwingSts = 1;
        RwingSts = 1;
        BwingSts = 1;
        pros::delay(200);
      } else if (RwingSts == 1 && LwingSts == 1) {
        robot::flapL.set_value(0);
        robot::flapR.set_value(0);
        LwingSts = 0;
        RwingSts = 0;
        BwingSts = 0;
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
    if (_1028A::robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) &&
        climb == neutral) {
      _1028A::robot::climb_set1.set_value(0);
      _1028A::robot::climb_set2.set_value(1);
      climb = up;
      pros::delay(500);
    } else if (_1028A::robot::master.get_digital(
                   pros::E_CONTROLLER_DIGITAL_DOWN) &&
               climb == up) {
      _1028A::robot::climb_set1.set_value(1);
      _1028A::robot::climb_set2.set_value(0);
      climb = down;
      pros::delay(500);
    } else if (_1028A::robot::master.get_digital(
                   pros::E_CONTROLLER_DIGITAL_DOWN) &&
               climb == down) {
      _1028A::robot::climb_set1.set_value(0);
      _1028A::robot::climb_set2.set_value(0);
      climb = neutral;
      pros::delay(500);
    }
    pros::delay(20);
  }
}

void _1028A::comp::driver::dataCTRL() {
  while (1) {
  }
}