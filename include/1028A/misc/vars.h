#include "display/lv_core/lv_style.h"
#include "main.h"
#pragma once

// Ui
extern bool batteryLow;
extern bool overTemp;
extern bool ports;
extern bool uiLock;
extern bool isDiskMag;
extern int macroStart;
extern int autonSelect;
extern lv_style_t style_ready;
extern lv_style_t style_notready;
extern lv_style_t style_standby;
extern lv_style_t style_blue;
extern lv_style_t style_red;
extern int currentScreen;
enum lastAutonSelect { goalSide, goalwp, loadingSide, loadingwp, Skills, null };
enum DiskColor { red, blue, none };
extern lastAutonSelect selection;
extern DiskColor diskColor;
extern int InitializeD;

// General
extern lv_obj_t *List;
extern lv_obj_t *autonBtn;
extern lv_obj_t *logsBtn;
extern lv_obj_t *macrosBtn;

// Auton Screen
extern lv_obj_t *autonPG;
extern lv_obj_t *summary;
extern lv_obj_t *summaryTxt;
extern lv_obj_t *blueBox;
extern lv_obj_t *RedNegWPBtn;
extern lv_obj_t *BlueNegWPBtn;
extern lv_obj_t *RedNegElimsBtn;
extern lv_obj_t *BlueNegElimstn;
extern lv_obj_t *redBox;
extern lv_obj_t *skills;

// Macros Screen
extern lv_obj_t *macrosPG;
extern lv_obj_t *logging;
extern lv_obj_t *odomReadout;
extern int startLogging;
extern int startReadout;

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
extern lv_obj_t *selectedAuto;