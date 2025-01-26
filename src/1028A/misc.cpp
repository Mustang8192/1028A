#include "1028A/misc.h"
#include "1028A/logger.h"
#include "1028A/ui.h"

void _1028A::misc::init(){
    _1028A::logger::init();
    lvgl_init();
    lv_init();
    pros::Task startUI(_1028A::ui::init);
}