#include "1028A/ui/callbacks.h"
#include "1028A/misc/logger.h"
#include "1028A/misc/robot.h"
#include "1028A/misc/vars.h"
#include "1028A/ui/screens.h"

lv_res_t _1028A::ui::callbacks::goalCB(lv_obj_t *list_btn) {
  if (selection == goalSide) {
    lv_obj_del(summary);
    selection = null;
    return LV_RES_OK;
  } else if (!uiLock) {
    autonSelect = 5;
    selection = goalSide;
    summary = lv_page_create(autonPG, NULL);
    lv_obj_set_size(summary, 150, 200);
    lv_obj_align(summary, NULL, LV_ALIGN_CENTER, 130, 0);
    lv_page_set_sb_mode(summary, LV_SB_MODE_OFF);
    lv_page_set_scrl_fit(summary, false, false);
    lv_page_set_scrl_layout(summary, LV_LAYOUT_PRETTY);
    summaryTxt = lv_label_create(summary, NULL);
    lv_label_set_text(summaryTxt,
                      "Rush 5:\n5 Triball\nNo WP task\nTriball Rush");
    return LV_RES_OK;
  } else {
    return LV_RES_OK;
  }
}

lv_res_t _1028A::ui::callbacks::loadingrushCB(lv_obj_t *list_btn) {
  if (selection == loadingSide) {
    lv_obj_del(summary);
    selection = null;
    return LV_RES_OK;
  } else if (!uiLock) {
    autonSelect = 3;
    selection = loadingSide;
    summary = lv_page_create(autonPG, NULL);
    lv_obj_set_size(summary, 150, 200);
    lv_obj_align(summary, NULL, LV_ALIGN_CENTER, -15, 0);
    lv_page_set_sb_mode(summary, LV_SB_MODE_OFF);
    lv_page_set_scrl_fit(summary, false, false);
    lv_page_set_scrl_layout(summary, LV_LAYOUT_PRETTY);
    summaryTxt = lv_label_create(summary, NULL);
    lv_label_set_text(summaryTxt, "Loading Side:\nN/A");
    return LV_RES_OK;
  } else {
    return LV_RES_OK;
  }
}

lv_res_t _1028A::ui::callbacks::ball6CB(lv_obj_t *list_btn) {
  if (selection == goalwp) {
    lv_obj_del(summary);
    selection = null;
    return LV_RES_OK;
  } else if (!uiLock) {
    autonSelect = 2;
    selection = goalwp;
    summary = lv_page_create(autonPG, NULL);
    lv_obj_set_size(summary, 150, 200);
    lv_obj_align(summary, NULL, LV_ALIGN_CENTER, 130, 0);
    lv_page_set_sb_mode(summary, LV_SB_MODE_OFF);
    lv_page_set_scrl_fit(summary, false, false);
    lv_page_set_scrl_layout(summary, LV_LAYOUT_PRETTY);
    summaryTxt = lv_label_create(summary, NULL);
    lv_label_set_text(summaryTxt, "6 Ball:\n6 in goal");
    return LV_RES_OK;
  } else {
    return LV_RES_OK;
  }
}

lv_res_t _1028A::ui::callbacks::loadingwpCB(lv_obj_t *list_btn) {
  if (selection == loadingwp) {
    lv_obj_del(summary);
    selection = null;
    return LV_RES_OK;
  } else if (!uiLock) {
    autonSelect = 4;
    selection = loadingwp;
    summary = lv_page_create(autonPG, NULL);
    lv_obj_set_size(summary, 150, 200);
    lv_obj_align(summary, NULL, LV_ALIGN_CENTER, -15, 0);
    lv_page_set_sb_mode(summary, LV_SB_MODE_OFF);
    lv_page_set_scrl_fit(summary, false, false);
    lv_page_set_scrl_layout(summary, LV_LAYOUT_PRETTY);
    summaryTxt = lv_label_create(summary, NULL);
    lv_label_set_text(summaryTxt, "Loading WP:\n1 Goal\n 2 Over");
    return LV_RES_OK;
  } else {
    return LV_RES_OK;
  }
}

lv_res_t _1028A::ui::callbacks::skillsCB(lv_obj_t *list_btn) {
  if (selection == Skills) {
    lv_obj_del(summary);
    selection = null;
    return LV_RES_OK;
  } else if (!uiLock) {
    autonSelect = 12;
    selection = Skills;
    summary = lv_page_create(autonPG, NULL);
    lv_obj_set_size(summary, 150, 200);
    lv_obj_align(summary, NULL, LV_ALIGN_CENTER, 130, 0);
    lv_page_set_sb_mode(summary, LV_SB_MODE_OFF);
    lv_page_set_scrl_fit(summary, false, false);
    lv_page_set_scrl_layout(summary, LV_LAYOUT_PRETTY);
    summaryTxt = lv_label_create(summary, NULL);
    lv_label_set_text(summaryTxt, "Skills:\nFlywheel");
    return LV_RES_OK;
  } else {
    return LV_RES_OK;
  }
}

lv_res_t _1028A::ui::callbacks::homeCB(lv_obj_t *list_btn) {
  currentScreen = 0;
  _1028A::logger::info("UI: Home Screen");
  if (!uiLock) {
    _1028A::ui::screens::homeScreen();
    return LV_RES_OK;
  } else {
    return LV_RES_OK;
  }
}

lv_res_t _1028A::ui::callbacks::autonCB(lv_obj_t *list_btn) {
  currentScreen = 1;
  _1028A::logger::info("UI: Auton Selector");
  if (!uiLock) {
    _1028A::ui::screens::autonScreen();
    return LV_RES_OK;
  } else {
    return LV_RES_OK;
  }
}

lv_res_t _1028A::ui::callbacks::macrosCB(lv_obj_t *list_btn) {
  currentScreen = 2;
  _1028A::logger::info("UI: Macros Screen");
  if (!uiLock) {
    _1028A::ui::screens::macrosScreen();
    return LV_RES_OK;
  } else {
    return LV_RES_OK;
  }
}

lv_res_t _1028A::ui::callbacks::do_nothingCB(lv_obj_t *list_btn) {
  return LV_RES_OK;
}

lv_res_t _1028A::ui::callbacks::lockCB(lv_obj_t *btn) {
  if (uiLock) {
    uiLock = false;
  } else if (!uiLock) {
    uiLock = true;
    pros::delay(5000);
    robot::inertial.reset();
    pros::delay(4000);
    robot::master.rumble("-.");
  }
  return LV_RES_OK;
}

lv_res_t _1028A::ui::callbacks::macroLogging(lv_obj_t *btn) {
  startLogging = 1;
  return LV_RES_OK;
}