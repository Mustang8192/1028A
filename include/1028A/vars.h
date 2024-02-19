#include "main.h"
#include <cstddef>
#include <time.h>

extern int kickeron;
extern int stickon;

// Climb
enum climbSts { up, down, neutral };
extern climbSts climb;

// Wings
enum wingSts { open, closed };
extern wingSts Lwing;
extern wingSts Rwing;

// logger
extern bool fileLog;
extern time_t robotStartTime;

// checks
extern bool ports;
extern bool overTemp;
extern bool batteryLow;

// ui
extern bool uiLock;
extern int autonSelect;
// Ui
extern lv_style_t style_meter;
extern lv_style_t style_box;
extern lv_style_t style_ready;
extern lv_style_t style_notready;
extern lv_style_t style_standby;
extern int currentScreen;
enum lastAutonSelect { goalSide, goalwp, loadingSide, loadingwp, Skills, null };
extern lastAutonSelect selection;

// General
extern lv_obj_t *List;
extern lv_obj_t *autonBtn;
extern lv_obj_t *logsBtn;
extern lv_obj_t *macrosBtn;

// Auton Screen
extern lv_obj_t *autonPG;
extern lv_obj_t *summary;
extern lv_obj_t *summaryTxt;
extern lv_obj_t *goal;
extern lv_obj_t *loading;
extern lv_obj_t *skills;

// Macros Screen
extern lv_obj_t *macrosPG;

// Home Screen
extern lv_obj_t *homePG;
extern lv_obj_t *status;
extern lv_obj_t *sepLine;
extern lv_obj_t *statusLabel;
extern lv_obj_t *driveSpd;
extern lv_obj_t *autonLabel;
extern lv_obj_t *autonsts;
extern lv_obj_t *batterysts;
extern lv_obj_t *overtempsts;
extern lv_obj_t *portbadsts;
extern lv_obj_t *batterystatus;
extern lv_obj_t *overtempstatus;
extern lv_obj_t *portbadstatus;
extern lv_obj_t *lockBtn;