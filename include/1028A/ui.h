#include "main.h"
#include "liblvgl/lvgl.h"

namespace _1028A::ui{
    void init();
    namespace utils{
        extern void createList(lv_obj_t *parent, void homeCB(lv_event_t * e), void posautoCB(lv_event_t * e), void negautoCB(lv_event_t * e), void macroCB(lv_event_t *e), void skillsCB(lv_event_t * e));
        extern void createBtn(lv_obj_t *name, lv_obj_t *location,
                      void callback(lv_event_t *e), lv_align_t alignment,
                      int offsetx, int offsety, int sizeX, int sizeY,
                      std::string text);
        extern void createLabel(lv_obj_t *label, lv_obj_t *location,
                        lv_align_t alignment, int offsetx, int offsety,
                        std::string text);
    }
    namespace checks{
        extern void overTemp();
        extern void badPort();
        extern void lowBattery();
        extern int isBadPort;
        extern int isOverTemp;
        extern int isLowBattery;
    }
    namespace screens{
        extern void home();
        extern void posauto();
        extern void negauto();
        extern void macro();
    }
    namespace callbacks{
        extern void homeCB(lv_event_t * e);
        extern void posautoCB(lv_event_t * e);
        extern void negautoCb(lv_event_t * e);
        extern void macroCB(lv_event_t * e);
        extern void skillsCB(lv_event_t * e);
        extern void donothingCB(lv_event_t * e);
        namespace autos{
            extern void posAuto1CB(lv_event_t * e);
            extern void posAuto2CB(lv_event_t * e);
            extern void posAuto3CB(lv_event_t * e);
            extern void posAuto4CB(lv_event_t * e);
            extern void posAuto5CB(lv_event_t * e);
            extern void posAuto6CB(lv_event_t * e);
            extern void negAuto1CB(lv_event_t * e);
            extern void negAuto2CB(lv_event_t * e);
            extern void negAuto3CB(lv_event_t * e);
            extern void negAuto4CB(lv_event_t * e);
            extern void negAuto5CB(lv_event_t * e);
            extern void negAuto6CB(lv_event_t * e);
        }
        namespace macros{
            extern void recordPosCB(lv_event_t * e);
            extern int recordPos;
        }
    }
}