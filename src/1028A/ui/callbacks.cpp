#include "1028A/ui/callbacks.h"
#include "1028A/logger.h"
#include "1028A/ui/screens.h"
#include "1028A/vars.h"

lv_res_t _1028A::ui::callbacks::goalCB(lv_obj_t *list_btn) {
  if (!uiLock) {
    autonSelect = 1;
    summary = lv_page_create(autonPG, NULL);
    lv_obj_set_size(summary, 150, 200);
    lv_obj_align(summary, NULL, LV_ALIGN_CENTER, 130, 0);
    lv_page_set_sb_mode(summary, LV_SB_MODE_OFF);
    lv_page_set_scrl_fit(summary, false, false);
    lv_page_set_scrl_layout(summary, LV_LAYOUT_PRETTY);
    summaryTxt = lv_label_create(summary, NULL);
    lv_label_set_text(summaryTxt, "Left Side:\n1 roller\n3 Disks");
    return LV_RES_OK;
  } else {
    return LV_RES_OK;
  }
}

lv_res_t _1028A::ui::callbacks::loadingCB(lv_obj_t *list_btn) {
  if (!uiLock) {
    autonSelect = 2;
    summary = lv_page_create(autonPG, NULL);
    lv_obj_set_size(summary, 150, 200);
    lv_obj_align(summary, NULL, LV_ALIGN_CENTER, 130, 0);
    lv_page_set_sb_mode(summary, LV_SB_MODE_OFF);
    lv_page_set_scrl_fit(summary, false, false);
    lv_page_set_scrl_layout(summary, LV_LAYOUT_PRETTY);
    summaryTxt = lv_label_create(summary, NULL);
    lv_label_set_text(summaryTxt, "Win Point:\n2 roller\n6 Disks");
    return LV_RES_OK;
  } else {
    return LV_RES_OK;
  }
}

lv_res_t _1028A::ui::callbacks::skillsCB(lv_obj_t *list_btn) {
  if (!uiLock) {
    autonSelect = 8;
    summary = lv_page_create(autonPG, NULL);
    lv_obj_set_size(summary, 150, 200);
    lv_obj_align(summary, NULL, LV_ALIGN_CENTER, 130, 0);
    lv_page_set_sb_mode(summary, LV_SB_MODE_OFF);
    lv_page_set_scrl_fit(summary, false, false);
    lv_page_set_scrl_layout(summary, LV_LAYOUT_PRETTY);
    summaryTxt = lv_label_create(summary, NULL);
    lv_label_set_text(summaryTxt, "Skills:\n4 roller\n20 Disks");
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
  }
  return LV_RES_OK;
}