#include "1028A/ui/utils.h"
#include "1028A/misc/logger.h"
#include "1028A/misc/misc.h"
#include "1028A/misc/vars.h"
#include "1028A/ui/callbacks.h"
#include "1028A/ui/screens.h"

void _1028A::ui::utils::stylesInit() {
  lv_style_copy(&style_box, &lv_style_plain);
  style_box.body.main_color = LV_COLOR_BLUE;
  style_box.body.grad_color = LV_COLOR_BLUE;
  style_box.body.border.color = LV_COLOR_BLUE;
  style_box.body.border.width = 1;
  style_box.body.border.opa = LV_OPA_50;
  style_box.body.radius = 0;
  style_box.text.color = LV_COLOR_WHITE;

  lv_style_copy(&style_ready, &lv_style_plain);
  style_ready.body.main_color = LV_COLOR_LIME;
  style_ready.body.grad_color = LV_COLOR_LIME;
  style_ready.body.border.color = LV_COLOR_LIME;
  style_ready.body.border.width = 1;
  style_ready.body.border.opa = LV_OPA_50;
  style_ready.body.radius = 0;
  style_ready.text.color = LV_COLOR_LIME;

  lv_style_copy(&style_notready, &lv_style_plain);
  style_notready.body.main_color = LV_COLOR_RED;
  style_notready.body.grad_color = LV_COLOR_RED;
  style_notready.body.border.color = LV_COLOR_RED;
  style_notready.body.border.width = 1;
  style_notready.body.border.opa = LV_OPA_50;
  style_notready.body.radius = 0;
  style_notready.text.color = LV_COLOR_RED;

  lv_style_copy(&style_standby, &lv_style_plain);
  style_standby.body.main_color = LV_COLOR_YELLOW;
  style_standby.body.grad_color = LV_COLOR_YELLOW;
  style_standby.body.border.color = LV_COLOR_YELLOW;
  style_standby.body.border.width = 1;
  style_standby.body.border.opa = LV_OPA_50;
  style_standby.body.radius = 0;
  style_standby.text.color = LV_COLOR_YELLOW;
}

void _1028A::ui::utils::createBtn(lv_obj_t *name, lv_obj_t *location,
                                  lv_res_t callback(lv_obj_t *btn),
                                  lv_align_t alignment, int offsetx,
                                  int offsety, int sizeX, int sizeY,
                                  std::string text) {
  name = lv_btn_create(location, NULL);
  lv_obj_align(name, NULL, alignment, offsetx, offsety);
  lv_obj_set_size(name, sizeX, sizeY);
  lv_cb_set_action(name, callback);

  lv_obj_t *BtnLbl = lv_label_create(name, NULL);
  lv_label_set_text(BtnLbl, text.c_str());
}

void _1028A::ui::utils::createBox(lv_obj_t *box, lv_obj_t *label,
                                  lv_obj_t *location, lv_align_t alignment,
                                  lv_align_t alignmentLbl, int offsetx,
                                  int offsety, int sizeX, int sizeY,
                                  int Lbloffsetx, int Lbloffsety,
                                  std::string text) {
  box = lv_obj_create(location, NULL);
  lv_obj_set_size(box, sizeX, sizeY);
  lv_obj_align(box, location, alignment, offsetx, offsety);
  lv_obj_set_style(box, &style_box);

  _1028A::ui::utils::createLabel(label, box, alignmentLbl, Lbloffsetx,
                                 Lbloffsety, text);
}

void _1028A::ui::utils::createLabel(lv_obj_t *label, lv_obj_t *location,
                                    lv_align_t alignment, int offsetx,
                                    int offsety, std::string text) {
  label = lv_label_create(location, NULL);
  lv_label_set_text(label, text.c_str());
  lv_obj_align(label, NULL, alignment, offsetx, offsety);
}

void _1028A::ui::utils::listInit(lv_res_t homeCB(lv_obj_t *list_btn),
                                 lv_res_t autonCB(lv_obj_t *list_btn),
                                 lv_res_t macrosCB(lv_obj_t *list_btn),
                                 lv_obj_t *parent, lv_obj_t *list) {
  list = lv_list_create(parent, NULL);
  lv_list_add(list, SYMBOL_HOME, "HOME", homeCB);
  autonBtn = lv_list_add(list, SYMBOL_PLAY, "AUTONS", autonCB);
  macrosBtn = lv_list_add(list, SYMBOL_REFRESH, "MACROS", macrosCB);
  lv_obj_set_width(list, 130);
}
void _1028A::ui::utils::doNothing() {}

void _1028A::ui::init() {
  _1028A::logger::info("Starting UI");
  pros::delay(1000);
  utils::stylesInit();
  _1028A::ui::screens::homeScreen();
}