#include "1028A/ui.h"
#include "1028A/auton.h"
#include "1028A/logger.h"
#include "liblvgl/core/lv_event.h"
#include "liblvgl/misc/lv_color.h"
#include "pros/screen.hpp"
#include "pros/apix.h"
#include "1028A/robot.h"

void _1028A::ui::init(){
    screens::home();
    lvgl_init();
}

void _1028A::ui::utils::createList(lv_obj_t *parent, void (*homeCB)(lv_event_t *), void (*posautoCB)(lv_event_t *), void (*negautoCB)(lv_event_t *), void (*macroCB)(lv_event_t *), void (*skillsCB)(lv_event_t *)){
    lv_obj_t *list = lv_list_create(parent);
    lv_obj_set_height(list, 240);
    lv_obj_set_width(list, 130);
    lv_obj_t * btn;
    btn = lv_list_add_btn(list, LV_SYMBOL_HOME, "Home");
    lv_obj_add_event_cb(btn, homeCB, LV_EVENT_CLICKED, NULL);
    btn = lv_list_add_btn(list, LV_SYMBOL_PLUS, "PAuto");
    lv_obj_add_event_cb(btn, posautoCB, LV_EVENT_CLICKED, NULL);
    btn = lv_list_add_btn(list, LV_SYMBOL_MINUS, "NAuto");
    lv_obj_add_event_cb(btn, negautoCB, LV_EVENT_CLICKED, NULL);
    btn = lv_list_add_btn(list, LV_SYMBOL_SETTINGS, "Macro");
    lv_obj_add_event_cb(btn, macroCB, LV_EVENT_CLICKED, NULL);
}

void _1028A::ui::utils::createBtn(lv_obj_t *name,
                                  lv_obj_t *location,
                                  void callback(lv_event_t *e),
                                  lv_align_t alignment, int offsetx,
                                  int offsety, int sizeX, int sizeY,
                                  std::string text) {
  name = lv_btn_create(location);
  lv_obj_align(name, alignment, offsetx, offsety);
  lv_obj_set_size(name, sizeX, sizeY);
  lv_obj_add_event_cb(name, callback, LV_EVENT_CLICKED, NULL);

  lv_obj_t *BtnLbl = lv_label_create(name);
  lv_label_set_text(BtnLbl, text.c_str());
}

int _1028A::ui::callbacks::macros::recordPos = 0;
void _1028A::ui::callbacks::macros::recordPosCB(lv_event_t *e){
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_CLICKED) {
        recordPos = 1;
    }
}
void _1028A::ui::callbacks::homeCB(lv_event_t * e){
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_CLICKED) {
        _1028A::ui::screens::home();
        _1028A::logger::info("Home Screen");
    }
}

void _1028A::ui::callbacks::posautoCB(lv_event_t * e){
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_CLICKED) {
        _1028A::ui::screens::posauto();
        _1028A::logger::info("PosAuto Screen");
    }
}

void _1028A::ui::callbacks::negautoCb(lv_event_t * e){
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_CLICKED) {
        _1028A::ui::screens::negauto();
        _1028A::logger::info("NegAuto Screen");
    }
}

void _1028A::ui::callbacks::macroCB(lv_event_t * e){
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_CLICKED) {
        _1028A::ui::screens::macro();
        _1028A::logger::info("Macro Screen");
    }
}

void _1028A::ui::callbacks::skillsCB(lv_event_t * e){
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_CLICKED) {
        _1028A::auton::autonSelect = 100;
        _1028A::logger::info("Skills Selected");
    }
}

void _1028A::ui::callbacks::donothingCB(lv_event_t * e){
    lv_event_code_t code = lv_event_get_code(e);
}

void _1028A::ui::callbacks::autos::posAuto1CB(lv_event_t * e){
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_CLICKED) {
        _1028A::auton::autonSelect = 1;
        _1028A::logger::info("PosAuto1 Selected");
    }
}

void _1028A::ui::callbacks::autos::posAuto2CB(lv_event_t * e){
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_CLICKED) {
        _1028A::auton::autonSelect = 2;
        _1028A::logger::info("PosAuto2 Selected");
    }
}

void _1028A::ui::callbacks::autos::posAuto3CB(lv_event_t * e){
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_CLICKED) {
        _1028A::auton::autonSelect = 3;
        _1028A::logger::info("PosAuto3 Selected");
    }
}

void _1028A::ui::callbacks::autos::posAuto4CB(lv_event_t * e){
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_CLICKED) {
        _1028A::auton::autonSelect = 4;
        _1028A::logger::info("PosAuto4 Selected");
    }
}

void _1028A::ui::callbacks::autos::posAuto5CB(lv_event_t * e){
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_CLICKED) {
        _1028A::auton::autonSelect = 5;
        _1028A::logger::info("PosAuto5 Selected");
    }
}

void _1028A::ui::callbacks::autos::posAuto6CB(lv_event_t * e){
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_CLICKED) {
        _1028A::auton::autonSelect = 6;
        _1028A::logger::info("PosAuto6 Selected");
    }
}

void _1028A::ui::callbacks::autos::negAuto1CB(lv_event_t * e){
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_CLICKED) {
        _1028A::auton::autonSelect = -1;
        _1028A::logger::info("NegAuto1 Selected");   }
}

void _1028A::ui::callbacks::autos::negAuto2CB(lv_event_t * e){
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_CLICKED) {
        _1028A::auton::autonSelect = -2;
        _1028A::logger::info("NegAuto2 Selected");
    }
}

void _1028A::ui::callbacks::autos::negAuto3CB(lv_event_t * e){
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_CLICKED) {
        _1028A::auton::autonSelect = -3;
        _1028A::logger::info("NegAuto3 Selected");
    }
}

void _1028A::ui::callbacks::autos::negAuto4CB(lv_event_t * e){
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_CLICKED) {
        _1028A::auton::autonSelect = -4;
        _1028A::logger::info("NegAuto4 Selected");
    }
}

void _1028A::ui::callbacks::autos::negAuto5CB(lv_event_t * e){
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_CLICKED) {
        _1028A::auton::autonSelect = -5;
        _1028A::logger::info("NegAuto5 Selected");
    }
}

void _1028A::ui::callbacks::autos::negAuto6CB(lv_event_t * e){
    lv_event_code_t code = lv_event_get_code(e);
    if(code == LV_EVENT_CLICKED) {
        _1028A::auton::autonSelect = -6;
        _1028A::logger::info("NegAuto6 Selected");
    }
}

void _1028A::ui::screens::home(){
    lv_obj_clean(lv_scr_act());
    lv_obj_t *scr = lv_obj_create(NULL);
    checks::badPort();
    checks::lowBattery();
    checks::overTemp();

    if ((auton::autonSelect != 0 && !checks::isBadPort && !checks::isLowBattery && !checks::isOverTemp &&
       pros::competition::is_connected())) {
    lv_obj_t * status = lv_label_create(scr);
    lv_obj_align(status, LV_ALIGN_CENTER, 80, -90);
    lv_label_set_recolor(status, true);
    lv_label_set_text(status, "#00FF00 READY");
  } else if ((!pros::competition::is_connected())) {
    lv_obj_t * status = lv_label_create(scr);
    lv_obj_align(status,  LV_ALIGN_CENTER, 87, -90);
    lv_label_set_recolor(status, true);
    lv_label_set_text(status, "#FFFF00 PRACTICE");
  } else {
    lv_obj_t * status = lv_label_create(scr);
    lv_obj_align(status,  LV_ALIGN_CENTER, 100, -90);
    lv_label_set_recolor(status, true);
    lv_label_set_text(status, "#FF0000 NOT READY");
  }
  if (!checks::isBadPort) {
    lv_obj_t *portbadsts = lv_label_create(scr);
    lv_obj_align(portbadsts, LV_ALIGN_CENTER, 180, -40);
    lv_label_set_recolor(portbadsts, true);
    lv_label_set_text(portbadsts, "#00FF00 PASSED");
  } else {
    lv_obj_t *portbadsts = lv_label_create(scr);
    lv_label_set_recolor(portbadsts, true);
    lv_obj_align(portbadsts,  LV_ALIGN_CENTER, 180, -40);
    lv_label_set_text(portbadsts, "#FF0000 FAILED");
  }

  if (!checks::isLowBattery) {
    lv_obj_t *batterysts = lv_label_create(scr);
    lv_label_set_recolor(batterysts, true);
    lv_obj_align(batterysts, LV_ALIGN_CENTER, 180, -10);
    lv_label_set_text(batterysts, "#00FF00 PASSED");
  } else {
    lv_obj_t *batterysts = lv_label_create(scr);
    lv_label_set_recolor(batterysts, true);
    lv_obj_align(batterysts, LV_ALIGN_CENTER, 180, -10);
    lv_label_set_text(batterysts, "#FF0000 FAILED");
  }

  if (!checks::isOverTemp) {
    lv_obj_t *overtempsts = lv_label_create(scr);
    lv_label_set_recolor(overtempsts, true);
    lv_obj_align(overtempsts, LV_ALIGN_CENTER, 180, 20);
    lv_label_set_text(overtempsts, "#00FF00 PASSED");
  } else {
    lv_obj_t *overtempsts = lv_label_create(scr);
    lv_label_set_recolor(overtempsts, true);
    lv_obj_align(overtempsts, LV_ALIGN_CENTER, 180, 20);
    lv_label_set_text(overtempsts, "#FF0000 FAILED");
  }

  if (auton::autonSelect != 0) {
    lv_obj_t *autonsts = lv_label_create(scr);
    lv_label_set_recolor(autonsts, true);
    lv_obj_align(autonsts, LV_ALIGN_CENTER, 178, 50);
    lv_label_set_text(autonsts, "#00FF00 SELECTED");
  } else {
    lv_obj_t *autonsts = lv_label_create(scr);
    lv_label_set_recolor(autonsts, true);
    lv_obj_align(autonsts, LV_ALIGN_CENTER, 157, 50);
    lv_label_set_text(autonsts, "#FF0000 NOT SELECTED");
  }

  lv_obj_t *autonLabel = lv_label_create(scr);
  lv_label_set_text(autonLabel, "Auton:");
  lv_obj_align(autonLabel, LV_ALIGN_CENTER, -57, 50);

  lv_obj_t *portbadstatus = lv_label_create(scr);
  lv_label_set_text(portbadstatus, "Port Check:");
  lv_obj_align(portbadstatus, LV_ALIGN_CENTER, -41, -40);

  lv_obj_t *batterystatus = lv_label_create(scr);
  lv_label_set_text(batterystatus, "Battery Check:");
  lv_obj_align(batterystatus, LV_ALIGN_CENTER, -30, -10);

  lv_obj_t *overtempstatus = lv_label_create(scr);
  lv_label_set_text(overtempstatus, "Over Temp Check:");
  lv_obj_align(overtempstatus, LV_ALIGN_CENTER, -17, 20);

  static lv_point_t line_points[] = {{100, 1}, {470, 1}};
  lv_obj_t *sepLine = lv_line_create(scr);
  lv_line_set_points(sepLine, line_points, 2);
  lv_obj_align(sepLine, LV_ALIGN_OUT_TOP_MID, 0, 50);

  lv_obj_t *statusLabel = lv_label_create(scr);
  lv_label_set_text(statusLabel, "Status:");
  lv_obj_align(statusLabel, LV_ALIGN_CENTER, 20, -90);

    utils::createList(scr, callbacks::donothingCB, callbacks::posautoCB, callbacks::negautoCb, callbacks::macroCB, callbacks::skillsCB);

    lv_scr_load(scr);
}
lv_style_t style;


void _1028A::ui::screens::posauto(){
    lv_obj_clean(lv_scr_act());
    lv_obj_t *scr = lv_obj_create(NULL);
    utils::createList(scr, callbacks::homeCB, callbacks::donothingCB, callbacks::negautoCb, callbacks::macroCB, callbacks::skillsCB);
    lv_obj_t *list = lv_list_create(scr);
    lv_obj_set_height(list, 240);
    lv_obj_set_width(list, 180);
    lv_obj_set_align(list, LV_ALIGN_CENTER);
    lv_obj_t * btn;
    btn = lv_list_add_btn(list, "1.", "Slot 1");
    lv_obj_add_event_cb(btn, callbacks::autos::posAuto1CB, LV_EVENT_CLICKED, NULL);
    btn = lv_list_add_btn(list, "2.", "Slot 2");
    lv_obj_add_event_cb(btn, callbacks::autos::posAuto2CB, LV_EVENT_CLICKED, NULL);
    btn = lv_list_add_btn(list, "3.", "Slot 3");
    lv_obj_add_event_cb(btn, callbacks::autos::posAuto3CB, LV_EVENT_CLICKED, NULL);
    btn = lv_list_add_btn(list, "4.", "Slot 4");
    lv_obj_add_event_cb(btn, callbacks::autos::posAuto4CB, LV_EVENT_CLICKED, NULL);
    btn = lv_list_add_btn(list, "5.", "Slot 5");
    lv_obj_add_event_cb(btn, callbacks::autos::posAuto5CB, LV_EVENT_CLICKED, NULL);
    btn = lv_list_add_btn(list, "6.", "Slot 6");
    lv_obj_add_event_cb(btn, callbacks::autos::posAuto6CB, LV_EVENT_CLICKED, NULL);
    lv_scr_load(scr);
}

void _1028A::ui::screens::negauto(){
    lv_obj_clean(lv_scr_act());
    lv_obj_t *scr = lv_obj_create(NULL);
    utils::createList(scr, callbacks::homeCB, callbacks::posautoCB, callbacks::donothingCB, callbacks::macroCB, callbacks::skillsCB);
    lv_obj_t *list = lv_list_create(scr);
    lv_obj_set_height(list, 240);
    lv_obj_set_width(list, 180);
    lv_obj_set_align(list, LV_ALIGN_CENTER);
    lv_obj_t * btn;
    btn = lv_list_add_btn(list, "1.", "Slot 1");
    lv_obj_add_event_cb(btn, callbacks::autos::negAuto1CB, LV_EVENT_CLICKED, NULL);
    btn = lv_list_add_btn(list, "2.", "Slot 2");
    lv_obj_add_event_cb(btn, callbacks::autos::negAuto2CB, LV_EVENT_CLICKED, NULL);
    btn = lv_list_add_btn(list, "3.", "Slot 3");
    lv_obj_add_event_cb(btn, callbacks::autos::negAuto3CB, LV_EVENT_CLICKED, NULL);
    btn = lv_list_add_btn(list, "4.", "Slot 4");
    lv_obj_add_event_cb(btn, callbacks::autos::negAuto4CB, LV_EVENT_CLICKED, NULL);
    btn = lv_list_add_btn(list, "5.", "Slot 5");
    lv_obj_add_event_cb(btn, callbacks::autos::negAuto5CB, LV_EVENT_CLICKED, NULL);
    btn = lv_list_add_btn(list, "6.", "Slot 6");
    lv_obj_add_event_cb(btn, callbacks::autos::negAuto6CB, LV_EVENT_CLICKED, NULL);
    lv_scr_load(scr);
}

void _1028A::ui::screens::macro(){
    lv_obj_clean(lv_scr_act());
    lv_obj_t *scr = lv_obj_create(NULL);
    utils::createList(scr, callbacks::homeCB, callbacks::posautoCB, callbacks::negautoCb, callbacks::donothingCB, callbacks::skillsCB);
    lv_obj_t *btn;
    utils::createBtn(btn, scr, callbacks::macros::recordPosCB, LV_ALIGN_CENTER, 0, 0, 100, 50, "Record Pos");
    lv_scr_load(scr);
}

int _1028A::ui::checks::isBadPort = 0;
int _1028A::ui::checks::isOverTemp = 0;
int _1028A::ui::checks::isLowBattery = 0;

void _1028A::ui::checks::lowBattery(){
    if(pros::battery::get_capacity() < 10){
        isLowBattery = 1;
        _1028A::logger::error("Low Battery");
    }
    else{
        isLowBattery = 0;
    }
}

void _1028A::ui::checks::badPort(){
    pros::c::v5_device_e_t LeftFrontcheck =
      pros::c::registry_get_plugged_type((leftFrontpt - 1));
    pros::c::v5_device_e_t LeftMidcheck = 
        pros::c::registry_get_plugged_type((leftMidpt - 1));
    pros::c::v5_device_e_t LeftBackcheck =
        pros::c::registry_get_plugged_type((leftBackpt - 1));
    pros::c::v5_device_e_t RightFrontcheck =
        pros::c::registry_get_plugged_type((rightFrontpt - 1));
    pros::c::v5_device_e_t RightMidcheck =
        pros::c::registry_get_plugged_type((rightMidpt - 1));
    pros::c::v5_device_e_t RightBackcheck =
        pros::c::registry_get_plugged_type((rightBackpt - 1));
    pros::c::v5_device_e_t Intakecheck =
        pros::c::registry_get_plugged_type((intakept - 1));
    pros::c::v5_device_e_t LBLcheck =
        pros::c::registry_get_plugged_type((LBLPort - 1));
    pros::c::v5_device_e_t LBRcheck =
        pros::c::registry_get_plugged_type((LBRPort - 1));
    pros::c::v5_device_e_t LBSCheck =
        pros::c::registry_get_plugged_type((LBSPort - 1));
    pros::c::v5_device_e_t VerticalCheck =
        pros::c::registry_get_plugged_type((Verticalpt - 1));
    pros::c::v5_device_e_t HorizontalCheck =
        pros::c::registry_get_plugged_type((Horizontalpt - 1));
    pros::c::v5_device_e_t InertialCheck =
        pros::c::registry_get_plugged_type((inertialpt - 1));
    pros::c::v5_device_e_t DistanceCheck =
        pros::c::registry_get_plugged_type((distancePort - 1));
    pros::c::v5_device_e_t OpticalCheck =
        pros::c::registry_get_plugged_type((opticalPort - 1));
    
    if(LeftFrontcheck != pros::c::E_DEVICE_MOTOR){
        isBadPort = 1;
        _1028A::logger::error("Left Front Motor Not Found");
    }
    else if(LeftMidcheck != pros::c::E_DEVICE_MOTOR){
        isBadPort = 1;
        _1028A::logger::error("Left Mid Motor Not Found");
    }
    else if(LeftBackcheck != pros::c::E_DEVICE_MOTOR){
        isBadPort = 1;
        _1028A::logger::error("Left Back Motor Not Found");
    }
    else if(RightFrontcheck != pros::c::E_DEVICE_MOTOR){
        isBadPort = 1;
        _1028A::logger::error("Right Front Motor Not Found");
    }
    else if(RightMidcheck != pros::c::E_DEVICE_MOTOR){
        isBadPort = 1;
        _1028A::logger::error("Right Mid Motor Not Found");
    }
    else if(RightBackcheck != pros::c::E_DEVICE_MOTOR){
        isBadPort = 1;
        _1028A::logger::error("Right Back Motor Not Found");
    }
    else if(Intakecheck != pros::c::E_DEVICE_MOTOR){
        isBadPort = 1;
        _1028A::logger::error("Intake Motor Not Found");
    }
    else if(LBLcheck != pros::c::E_DEVICE_MOTOR){
        isBadPort = 1;
        _1028A::logger::error("LB Left Motor Not Found");
    }
    else if(LBRcheck != pros::c::E_DEVICE_MOTOR){
        isBadPort = 1;
        _1028A::logger::error("LB Right Motor Not Found");
    }
    else if(LBSCheck != pros::c::E_DEVICE_ROTATION){
        isBadPort = 1;
        _1028A::logger::error("LB Sensor Not Found");
    }
    else if(VerticalCheck != pros::c::E_DEVICE_ROTATION){
        isBadPort = 1;
        _1028A::logger::error("Vertical Tracking Wheel Not Found");
    }
    else if(HorizontalCheck != pros::c::E_DEVICE_ROTATION){
        isBadPort = 1;
        _1028A::logger::error("Horizontal Tracking Wheel Not Found");
    }
    else if(InertialCheck != pros::c::E_DEVICE_IMU){
        isBadPort = 1;
        _1028A::logger::error("Inertial Sensor Not Found");
    }
    else if(OpticalCheck != pros::c::E_DEVICE_OPTICAL){
        isBadPort = 1;
        _1028A::logger::error("Optical Sensor Not Found");
    }
    else{
        isBadPort = 0;
    }   
}

void _1028A::ui::checks::overTemp(){
    if(_1028A::robot::leftFront.is_over_temp()){
        isOverTemp = 1;
        _1028A::logger::warn("Left Front Motor Over Temp");
    }
    if(_1028A::robot::leftMid.is_over_temp()){
        isOverTemp = 1;
        _1028A::logger::warn("Left Mid Motor Over Temp");
    }
    if(_1028A::robot::leftBack.is_over_temp()){
        isOverTemp = 1;
        _1028A::logger::warn("Left Back Motor Over Temp");
    }
    if(_1028A::robot::rightFront.is_over_temp()){
        isOverTemp = 1;
        _1028A::logger::warn("Right Front Motor Over Temp");
    }
    if(_1028A::robot::rightMid.is_over_temp()){
        isOverTemp = 1;
        _1028A::logger::warn("Right Mid Motor Over Temp");
    }
    if(_1028A::robot::rightBack.is_over_temp()){
        isOverTemp = 1;
        _1028A::logger::warn("Right Back Motor Over Temp");
    }
    if(_1028A::robot::intake.is_over_temp()){
        isOverTemp = 1;
        _1028A::logger::warn("Intake Motor Over Temp");
    }
    if(_1028A::robot::LBL.is_over_temp()){
        isOverTemp = 1;
        _1028A::logger::warn("LBL Motor Over Temp");
    }
    if(_1028A::robot::LBR.is_over_temp()){
        isOverTemp = 1;
        _1028A::logger::warn("LBR Motor Over Temp");
    }
}