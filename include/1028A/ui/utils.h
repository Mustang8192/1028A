#include "main.h"

namespace _1028A::ui::utils {
extern void stylesInit();

extern void createBtn(lv_obj_t *name, lv_obj_t *location,
                      lv_res_t callback(lv_obj_t *btn), lv_align_t alignment,
                      int offsetx, int offsety, int sizeX, int sizeY,
                      std::string text);

extern void createLabel(lv_obj_t *label, lv_obj_t *location,
                        lv_align_t alignment, int offsetx, int offsety,
                        std::string text);

extern void createBox(lv_obj_t *box, lv_obj_t *label, lv_obj_t *location,
                      lv_align_t alignment, lv_align_t alignmentLbl,
                      int offsetx, int offsety, int sizeX, int sizeY,
                      int Lbloffsetx, int Lbloffsety, std::string text);

extern void listInit(lv_res_t homeCB(lv_obj_t *list_btn),
                     lv_res_t autonCB(lv_obj_t *list_btn),
                     lv_res_t macrosCB(lv_obj_t *list_btn), lv_obj_t *parent,
                     lv_obj_t *list);

extern void doNothing();
} // namespace _1028A::ui::utils

namespace _1028A::ui {
extern void init();
}