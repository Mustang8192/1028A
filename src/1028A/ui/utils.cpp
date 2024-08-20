#include "1028A/ui/utils.h"
#include "1028A/misc/logger.h"
#include "1028A/misc/misc.h"
#include "1028A/misc/vars.h"
#include "1028A/ui/callbacks.h"
#include "1028A/ui/screens.h"
#include "display/lv_core/lv_obj.h"

void _1028A::ui::utils::stylesInit() {

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

  lv_style_copy(&style_blue, &lv_style_plain);
  style_blue.text.color = LV_COLOR_BLUE;
  style_blue.body.radius = 10;
  style_blue.body.main_color = LV_COLOR_BLUE;

  lv_style_copy(&style_red, &lv_style_plain);
  style_red.text.color = LV_COLOR_RED;
  style_red.body.radius = 10;
  style_red.body.main_color = LV_COLOR_RED;
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

void _1028A::ui::utils::createBtn(lv_obj_t *name, lv_style_t &style,
                                  lv_obj_t *location,
                                  lv_res_t callback(lv_obj_t *btn),
                                  lv_align_t alignment, int offsetx,
                                  int offsety, int sizeX, int sizeY,
                                  std::string text) {
  name = lv_btn_create(location, NULL);
  lv_obj_align(name, NULL, alignment, offsetx, offsety);
  lv_obj_set_size(name, sizeX, sizeY);
  lv_cb_set_action(name, callback);
  lv_obj_set_style(name, &style);

  lv_obj_t *BtnLbl = lv_label_create(name, NULL);
  lv_label_set_text(BtnLbl, text.c_str());
}

void _1028A::ui::utils::createBox(lv_obj_t *box, lv_obj_t *location,
                                  lv_align_t alignment, int offsetx,
                                  int offsety, int sizeX, int sizeY) {
  box = lv_page_create(location, NULL);
  lv_obj_set_size(box, sizeX, sizeY);
  lv_obj_align(box, NULL, alignment, offsetx, offsety);
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