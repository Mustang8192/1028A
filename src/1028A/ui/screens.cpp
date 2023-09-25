#include "1028A/ui/screens.h"
#include "1028A/ui/callbacks.h"
#include "1028A/ui/utils.h"
#include "1028A/vars.h"

void _1028A::ui::screens::autonScreen() {
  autonPG = lv_obj_create(NULL, NULL);

  utils::createBtn(Left, autonPG, _1028A::ui::callbacks::LeftCB,
                   LV_ALIGN_CENTER, -11, -55, 100, 50, "Left Side");
  utils::createBtn(Right, autonPG, _1028A::ui::callbacks::solowpCB,
                   LV_ALIGN_CENTER, -11, 0, 100, 50, "Solo WP");
  utils::createBtn(solowp, autonPG, _1028A::ui::callbacks::RightCB,
                   LV_ALIGN_CENTER, -11, 55, 100, 50, "Right Side");
  utils::createBtn(skills, autonPG, _1028A::ui::callbacks::skillsCB,
                   LV_ALIGN_CENTER, -11, 110, 100, 50, "Skills");
  utils::listInit(
      _1028A::ui::callbacks::homeCB, _1028A::ui::callbacks::do_nothingCB,
      _1028A::ui::callbacks::motorsCB, _1028A::ui::callbacks::sensorsCB,
      _1028A::ui::callbacks::settingsCB, autonPG, List);

  lv_scr_load(autonPG);
}

void _1028A::ui::screens::motorsScreen() {
  motorsPG = lv_obj_create(NULL, NULL);

  utils::createLnMeter(Drivemeter, motorsPG, LV_ALIGN_IN_TOP_MID, -20, 20, 120,
                       120, 240, 31, 0, 100);
  utils::createLnMeter(Rightfronttemp, motorsPG, LV_ALIGN_IN_TOP_RIGHT, -5, 10,
                       70, 70, 240, 15, 0, 100);
  utils::createLnMeter(Leftfronttemp, motorsPG, LV_ALIGN_IN_TOP_RIGHT, -90, 10,
                       70, 70, 240, 15, 0, 100);
  utils::createLnMeter(Rightmidtemp, motorsPG, LV_ALIGN_IN_TOP_RIGHT, -5, 80,
                       70, 70, 240, 15, 0, 100);
  utils::createLnMeter(Leftmidtemp, motorsPG, LV_ALIGN_IN_TOP_RIGHT, -90, 80,
                       70, 70, 240, 15, 0, 100);
  utils::createLnMeter(Rightbacktemp, motorsPG, LV_ALIGN_IN_TOP_RIGHT, -5, 160,
                       70, 70, 240, 15, 0, 100);
  utils::createLnMeter(Leftbacktemp, motorsPG, LV_ALIGN_IN_TOP_RIGHT, -90, 160,
                       70, 70, 240, 15, 0, 100);
  utils::createLnMeter(catatemp, motorsPG, LV_ALIGN_IN_TOP_RIGHT, -180, 140, 70,
                       70, 240, 15, 0, 100);
  utils::createLnMeter(intaketemp, motorsPG, LV_ALIGN_IN_TOP_RIGHT, -275, 140,
                       70, 70, 240, 15, 0, 100);
  utils::listInit(_1028A::ui::callbacks::homeCB, _1028A::ui::callbacks::autonCB,
                  _1028A::ui::callbacks::do_nothingCB,
                  _1028A::ui::callbacks::sensorsCB,
                  _1028A::ui::callbacks::settingsCB, motorsPG, List);
  /*
    Leftfronttempval = leftFront.get_temperature();
    Rightfronttempval = rightFront.get_temperature();
    Leftbacktempval = leftBack.get_temperature();
    Rightbacktempval = rightBack.get_temperature();
    Leftmidtempval = leftMid.get_temperature();
    Rightmidtempval = rightMid.get_temperature();
    driveSpdval = (fabs(leftFront.get_target_velocity()) +
                   fabs(rightFront.get_target_velocity()) +
                   fabs(leftBack.get_target_velocity()) +
                   fabs(rightBack.get_target_velocity()) +
                   fabs(leftMid.get_target_velocity()) +
                   fabs(rightMid.get_target_velocity())) /
                  6;
    lv_lmeter_set_value(Leftfronttemp, Leftfronttempval);
    lv_lmeter_set_value(Rightfronttemp, Rightfronttempval);
    lv_lmeter_set_value(Leftbacktemp, Leftbacktempval);
    lv_lmeter_set_value(Rightbacktemp, Rightbacktempval);
    lv_lmeter_set_value(Leftmidtemp, Leftmidtempval);
    lv_lmeter_set_value(Rightmidtemp, Rightmidtempval);
    lv_lmeter_set_value(Drivemeter, driveSpdval);
    */
  lv_scr_load(motorsPG);
}

void _1028A::ui::screens::sensorsScreen() {
  sensorsPG = lv_obj_create(NULL, NULL);

  utils::createBox(InertialBox, InertialLabel, sensorsPG, LV_ALIGN_CENTER,
                   LV_ALIGN_IN_TOP_LEFT, 55, 66, 300, 100, 0, 0, "Inertial:");

  utils::createBox(LimitBoxA, LimitA, sensorsPG, LV_ALIGN_IN_TOP_LEFT,
                   LV_ALIGN_IN_TOP_LEFT, 145, 10, 105, 31, 0, 0, "Limit A:");

  utils::createBox(LimitBoxB, LimitB, sensorsPG, LV_ALIGN_IN_TOP_LEFT,
                   LV_ALIGN_IN_TOP_LEFT, 145, 45, 105, 31, 0, 0, "Limit B:");

  utils::createBox(cataEncBox, cataEncLabel, sensorsPG, LV_ALIGN_CENTER,
                   LV_ALIGN_IN_TOP_LEFT, -20, -10, 150, 45, 0, 0, "Cata:");

  utils::createBox(posEncBox, posEncLabel, sensorsPG, LV_ALIGN_CENTER,
                   LV_ALIGN_IN_TOP_LEFT, 145, -10, 150, 45, 0, 0, "Rota:");

  utils::createBox(OptiBox, OptiLabel, sensorsPG, LV_ALIGN_IN_TOP_RIGHT,
                   LV_ALIGN_IN_TOP_LEFT, -19, 5, 190, 80, 0, 0, "Optical:");
  utils::listInit(_1028A::ui::callbacks::homeCB, _1028A::ui::callbacks::autonCB,
                  _1028A::ui::callbacks::motorsCB,
                  _1028A::ui::callbacks::do_nothingCB,
                  _1028A::ui::callbacks::settingsCB, sensorsPG, List);
  lv_scr_load(sensorsPG);
}

void _1028A::ui::screens::settingsScreen() {
  settingsPG = lv_obj_create(NULL, NULL);
  // utils::createBtn(graphingBtn, settingsPG, callbacks::graphingCB,
  //                  LV_ALIGN_CENTER, 75, -60, 100, 50, "Graphing");
  utils::listInit(_1028A::ui::callbacks::homeCB, _1028A::ui::callbacks::autonCB,
                  _1028A::ui::callbacks::motorsCB,
                  _1028A::ui::callbacks::sensorsCB,
                  _1028A::ui::callbacks::do_nothingCB, settingsPG, List);

  lv_scr_load(settingsPG);
}
void _1028A::ui::screens::homeScreen() {
  homePG = lv_obj_create(NULL, NULL);
  if ((autonSelect != 0 && ports && !batteryLow && !overTemp &&
       pros::competition::is_connected())) {
    status = lv_label_create(homePG, NULL);
    lv_label_set_style(status, &style_ready);
    lv_obj_align(status, homePG, LV_ALIGN_CENTER, 80, -90);
    lv_label_set_text(status, "READY");
  } else if ((!pros::competition::is_connected())) {
    status = lv_label_create(homePG, NULL);
    lv_label_set_style(status, &style_standby);
    lv_obj_align(status, homePG, LV_ALIGN_CENTER, 80, -90);
    lv_label_set_text(status, "PRACTICE");
  } else {
    status = lv_label_create(homePG, NULL);
    lv_label_set_style(status, &style_notready);
    lv_obj_align(status, homePG, LV_ALIGN_CENTER, 80, -90);
    lv_label_set_text(status, "NOT READY");
  }
  if (ports) {
    portbadsts = lv_label_create(homePG, NULL);
    lv_label_set_style(portbadsts, &style_ready);
    lv_obj_align(portbadsts, homePG, LV_ALIGN_CENTER, 180, -40);
    lv_label_set_text(portbadsts, "PASSED");
  } else {
    portbadsts = lv_label_create(homePG, NULL);
    lv_label_set_style(portbadsts, &style_notready);
    lv_obj_align(portbadsts, homePG, LV_ALIGN_CENTER, 180, -40);
    lv_label_set_text(portbadsts, "FAILED");
  }

  if (!batteryLow) {
    batterysts = lv_label_create(homePG, NULL);
    lv_label_set_style(batterysts, &style_ready);
    lv_obj_align(batterysts, homePG, LV_ALIGN_CENTER, 180, -10);
    lv_label_set_text(batterysts, "PASSED");
  } else {
    batterysts = lv_label_create(homePG, NULL);
    lv_label_set_style(batterysts, &style_notready);
    lv_obj_align(batterysts, homePG, LV_ALIGN_CENTER, 180, -10);
    lv_label_set_text(batterysts, "FAILED");
  }

  if (!overTemp) {
    overtempsts = lv_label_create(homePG, NULL);
    lv_label_set_style(overtempsts, &style_ready);
    lv_obj_align(overtempsts, homePG, LV_ALIGN_CENTER, 180, 20);
    lv_label_set_text(overtempsts, "PASSED");
  } else {
    overtempsts = lv_label_create(homePG, NULL);
    lv_label_set_style(overtempsts, &style_notready);
    lv_obj_align(overtempsts, homePG, LV_ALIGN_CENTER, 180, 20);
    lv_label_set_text(overtempsts, "FAILED");
  }

  if (autonSelect != 0) {
    autonsts = lv_label_create(homePG, NULL);
    lv_label_set_style(autonsts, &style_ready);
    lv_obj_align(autonsts, homePG, LV_ALIGN_CENTER, 165, 50);
    lv_label_set_text(autonsts, "SELECTED");
  } else {
    autonsts = lv_label_create(homePG, NULL);
    lv_label_set_style(autonsts, &style_notready);
    lv_obj_align(autonsts, homePG, LV_ALIGN_CENTER, 115, 50);
    lv_label_set_text(autonsts, "NOT SELECTED");
  }

  autonLabel = lv_label_create(homePG, NULL);
  lv_label_set_text(autonLabel, "Auton:");
  lv_obj_align(autonLabel, homePG, LV_ALIGN_CENTER, -70, 50);

  portbadstatus = lv_label_create(homePG, NULL);
  lv_label_set_text(portbadstatus, "Port Check:");
  lv_obj_align(portbadstatus, homePG, LV_ALIGN_CENTER, -44, -40);

  batterystatus = lv_label_create(homePG, NULL);
  lv_label_set_text(batterystatus, "Battery Check:");
  lv_obj_align(batterystatus, homePG, LV_ALIGN_CENTER, -30, -10);

  overtempstatus = lv_label_create(homePG, NULL);
  lv_label_set_text(overtempstatus, "Over Temp Check:");
  lv_obj_align(overtempstatus, homePG, LV_ALIGN_CENTER, -15, 20);

  static lv_point_t line_points[] = {{100, 1}, {500, 1}};
  sepLine = lv_line_create(homePG, NULL);
  lv_line_set_points(sepLine, line_points, 2);
  lv_obj_align(sepLine, homePG, LV_ALIGN_OUT_TOP_MID, 0, 50);

  statusLabel = lv_label_create(homePG, NULL);
  lv_label_set_text(statusLabel, "Status:");
  lv_obj_align(statusLabel, homePG, LV_ALIGN_CENTER, 20, -90);

  utils::createBtn(lockBtn, homePG, callbacks::lockCB, LV_ALIGN_IN_BOTTOM_RIGHT,
                   40, 40, 75, 30, "Lock");

  utils::listInit(callbacks::do_nothingCB, callbacks::autonCB,
                  callbacks::motorsCB, callbacks::sensorsCB,
                  _1028A::ui::callbacks::settingsCB, homePG, List);
  lv_scr_load(homePG);
}