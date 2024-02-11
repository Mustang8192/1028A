#include "1028A/vars.h"

int kickeron = 0;
int stickon = 0;

// Climb
climbSts climb = neutral;

// Wings
int LwingSts = 0;
int RwingSts = 0;
int BwingSts = 0;

// logger
bool fileLog = false;
time_t robotStartTime = 0;

// checks
bool ports = true;
bool overTemp = false;
bool batteryLow = false;

// ui
bool uiLock = false;
int autonSelect = 0;
// UI
lv_style_t style_meter;
lv_style_t style_box;
lv_style_t style_ready;
lv_style_t style_notready;
lv_style_t style_standby;
int currentScreen = 0;
lastAutonSelect selection = null;

// General
lv_obj_t *List;
lv_obj_t *autonBtn;
lv_obj_t *logsBtn;
lv_obj_t *macrosBtn;

// Macros Screen
lv_obj_t *autonPG;
lv_obj_t *summary;
lv_obj_t *summaryTxt;
lv_obj_t *goal;
lv_obj_t *loading;
lv_obj_t *skills;

// Sensor Screen
lv_obj_t *macrosPG;

// Home Screen
lv_obj_t *homePG;
lv_obj_t *status;
lv_obj_t *sepLine;
lv_obj_t *statusLabel;
lv_obj_t *autonLabel;
lv_obj_t *autonsts;
lv_obj_t *batterysts;
lv_obj_t *overtempsts;
lv_obj_t *portbadsts;
lv_obj_t *batterystatus;
lv_obj_t *overtempstatus;
lv_obj_t *portbadstatus;
lv_obj_t *lockBtn;