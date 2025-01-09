#include "1028A/driver.h"
#include "1028A/robot.h"
#include "1028A/legacy.h"
#include "pros/misc.h"
#include "pros/rtos.hpp"

void _1028A::driver::driveCTRL(){
    _1028A::robot::leftMtrs.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    _1028A::robot::rightMtrs.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    _1028A::robot::LB.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    while (1){
        int power = _1028A::robot::master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int turn = _1028A::robot::master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        _1028A::robot::chassis.arcade(power, turn);
        pros::delay(10);
    }
}

void _1028A::driver::intakeCTRL(){
    while (1){
        if(_1028A::robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
            _1028A::robot::intake.move(-127);
        }
        else if (_1028A::robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
            _1028A::robot::intake.move(127);
        }
        else{
            _1028A::robot::intake.move(0);
        }

        pros::delay(10);
    }
}

void _1028A::driver::mogoCTRL(){
    int status = 0;
    while (1){
        if (_1028A::robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && status==0){
            _1028A::robot::mogo.set_value(1);
            status = 1;
            pros::delay(300);
        }
        else if (_1028A::robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && status==1){
            _1028A::robot::mogo.set_value(0);
            status = 0;
            pros::delay(300);
        }
        pros::delay(10);
    }
}

int targetVal = 0;

void _1028A::driver::lbmacro(){
    while (1){
        if (_1028A::robot::master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
            targetVal=65;
        }

        pros::delay(10);
    }
    
}

void moveLB(){
    float SensorCurrentValue;
    float error;
    float lastError = 0;

    float kP = 1;
    float kI = 0;
    float kD = 0;
    double timeExit = 0;
    double stratTime = pros::millis();
    _1028A::robot::leftMtrs.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    _1028A::robot::rightMtrs.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    while (1){
        SensorCurrentValue = _1028A::robot::LBS.get_position();
        error = targetVal - SensorCurrentValue;
        
        int power = _1028A::legacy::math(error, lastError, kP, kI, kD, 127);

        if (power<5 or error<=1){
            _1028A::robot::LB.move(0);
        }
        else{
            _1028A::robot::LB.move(power);
        }
        lastError = error;
        pros::delay(10);
    }

}