#include "1028A/ui/callbacks.h"
#include "1028A/misc/logger.h"
#include "1028A/misc/robot.h"
#include "1028A/misc/vars.h"
#include "1028A/ui/screens.h"
#include "pros/rtos.hpp"

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
    lv_label_set_text(summaryTxt, "Skills:");
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
    pros::delay(2000);
    robot::inertial.reset(false);
    pros::delay(3000);
    robot::master.rumble("-.");
  }
  return LV_RES_OK;
}

lv_res_t _1028A::ui::callbacks::macroLogging(lv_obj_t *btn) {
  if (startLogging) {
    startLogging = 0;
  } else if (!startLogging) {
    startLogging = 1;
  }
  return LV_RES_OK;
}

lv_res_t _1028A::ui::callbacks::macroReadout(lv_obj_t *btn) {
  if (startReadout) {
    startReadout = 0;
  } else if (!startReadout) {
    startReadout = 1;
  }
  return LV_RES_OK;
}

lv_res_t _1028A::ui::callbacks::RedLCB(lv_obj_t *btn) {
  autonSelect = 2;
  return LV_RES_OK;
};

lv_res_t _1028A::ui::callbacks::RedRCB(lv_obj_t *btn) {
  autonSelect = 8;
  return LV_RES_OK;
}

lv_res_t _1028A::ui::callbacks::BlueLCB(lv_obj_t *btn) {
  autonSelect = 1;
  return LV_RES_OK;
};

lv_res_t _1028A::ui::callbacks::BlueRCB(lv_obj_t *btn) {
  autonSelect = 4;
  return LV_RES_OK;
};