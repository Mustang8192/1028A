#include "1028A/vars.h"

// logger
bool fileLog = false;
time_t robotStartTime = 0;

// checks
bool ports = true;
bool overTemp = false;
bool batteryLow = false;

// robot
ptoState PTO = ptoState::drive;

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

// General
lv_obj_t *List;
lv_obj_t *autonBtn;
lv_obj_t *sensorsBtn;
lv_obj_t *motorsBtn;
lv_obj_t *settingsBtn;

// Auton Screen
lv_obj_t *autonPG;
lv_obj_t *summary;
lv_obj_t *summaryTxt;
lv_obj_t *Left;
lv_obj_t *Right;
lv_obj_t *solowp;
lv_obj_t *skills;

// Motors Screen
lv_obj_t *motorsPG;
lv_obj_t *Drivemeter;
lv_obj_t *Leftfronttemp;
lv_obj_t *Leftmidtemp;
lv_obj_t *Leftbacktemp;
lv_obj_t *Rightfronttemp;
lv_obj_t *Rightmidtemp;
lv_obj_t *Rightbacktemp;
lv_obj_t *catatemp;
lv_obj_t *intaketemp;
double Leftfronttempval;
double Leftmidtempval;
double Leftbacktempval;
double Rightfronttempval;
double Rightmidtempval;
double Rightbacktempval;
double driveSpdval;

// Sensor Screen
lv_obj_t *sensorsPG;
lv_obj_t *InertialBox;
lv_obj_t *InertialLabelRot;
lv_obj_t *InertialLabelHeading;
lv_obj_t *InertialLabelPitch;
lv_obj_t *InertialLabelRoll;
lv_obj_t *InertialLabelYaw;
lv_obj_t *InertialLabelAccelX;
lv_obj_t *InertialLabelAccelY;
lv_obj_t *InertialLabelAccelZ;
lv_obj_t *InertialLabel;
lv_obj_t *LimitBoxA;
lv_obj_t *LimitA;
lv_obj_t *LimitALabel;
lv_obj_t *LimitBoxB;
lv_obj_t *LimitB;
lv_obj_t *LimitBLabel;
lv_obj_t *cataEncBox;
lv_obj_t *cataEncLabel;
lv_obj_t *cataEncLabelAngle;
lv_obj_t *posEncBox;
lv_obj_t *posEncLabel;
lv_obj_t *posEncLabelRot;
lv_obj_t *OptiBox;
lv_obj_t *OptiLabel;

// Settings Screen
lv_obj_t *settingsPG;
lv_obj_t *grafanaBtn;
lv_obj_t *autonRecorderBtn;
lv_obj_t *tensorflowliteBtn;
lv_obj_t *odomDebugBtn;
lv_obj_t *graphingBtn;
lv_obj_t *rosBtn;

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