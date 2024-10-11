#include "1028A/misc/vars.h"

// ui
bool batteryLow = false;
bool overTemp = false;
bool ports = true;
bool uiLock = false;
bool isDiskMag = false;
int macroStart = 0;
int autonSelect = 0;
lv_style_t style_ready;
lv_style_t style_notready;
lv_style_t style_standby;
lv_style_t style_blue;
lv_style_t style_red;
int currentScreen = 0;
lastAutonSelect selection = null;
DiskColor diskColor = none;

// General
lv_obj_t *List;
lv_obj_t *autonBtn;
lv_obj_t *logsBtn;
lv_obj_t *macrosBtn;

// Auton Screen
lv_obj_t *autonPG;
lv_obj_t *summary;
lv_obj_t *summaryTxt;
lv_obj_t *blueBox;
lv_obj_t *RedNegWPBtn;
lv_obj_t *BlueNegWPBtn;
lv_obj_t *RedNegElimsBtn;
lv_obj_t *BlueNegElimstn;
lv_obj_t *redBox;
lv_obj_t *skills;

// Macro Screen
lv_obj_t *macrosPG;
lv_obj_t *logging;
lv_obj_t *odomReadout;
int startLogging = 0;
int startReadout = 0;

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
lv_obj_t *selectedAuto;