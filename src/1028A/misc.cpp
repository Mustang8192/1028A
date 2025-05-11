#include "1028A/misc.h"
#include "1028A/logger.h"
#include "1028A/robot.h"
#include "1028A/ui.h"

void _1028A::misc::init(){
    _1028A::logger::init();
    lvgl_init();
    lv_init();
    pros::Task startUI(_1028A::ui::init);
}

void _1028A::misc::waitForCalibrate(){
    while (1){
        if (_1028A::robot::CaliSwitch.get_value()){
            _1028A::robot::master.rumble("-");
            _1028A::logger::info("Calibrating");
            _1028A::robot::chassis.calibrate();
            _1028A::robot::master.rumble("-.");
            break;
        }
        pros::delay(20);
    }
}