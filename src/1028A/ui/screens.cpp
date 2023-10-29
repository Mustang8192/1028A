#include "1028A/ui/screens.h"
#include "1028A/misc.h"
#include "1028A/ui/callbacks.h"
#include "1028A/ui/utils.h"
#include "1028A/vars.h"

void _1028A::ui::screens::autonScreen() {
  autonPG = lv_obj_create(NULL, NULL);

  utils::createBtn(goal, autonPG, _1028A::ui::callbacks::goalCB,
                   LV_ALIGN_CENTER, -11, -55, 100, 50, "Goal Side");
  utils::createBtn(loading, autonPG, _1028A::ui::callbacks::loadingCB,
                   LV_ALIGN_CENTER, -11, 0, 100, 50, "Loading WP");
  utils::createBtn(skills, autonPG, _1028A::ui::callbacks::skillsCB,
                   LV_ALIGN_CENTER, -11, 110, 100, 50, "Skills");
  utils::listInit(_1028A::ui::callbacks::homeCB,
                  _1028A::ui::callbacks::do_nothingCB,
                  _1028A::ui::callbacks::macrosCB, autonPG, List);

  lv_scr_load(autonPG);
}

void _1028A::ui::screens::macrosScreen() {
  macrosPG = lv_obj_create(NULL, NULL);

  utils::listInit(_1028A::ui::callbacks::homeCB, _1028A::ui::callbacks::autonCB,
                  _1028A::ui::callbacks::do_nothingCB, macrosPG, List);
  lv_scr_load(macrosPG);
}

void _1028A::ui::screens::homeScreen() {
  _1028A::utils::checks();
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
                  callbacks::macrosCB, homePG, List);
  lv_scr_load(homePG);
}