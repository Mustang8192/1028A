#include "main.h"
#include <time.h>

// logger
extern bool fileLog;
extern time_t robotStartTime;

// checks
extern bool ports;
extern bool overTemp;
extern bool batteryLow;

// robot
enum class ptoState { drive, cata };
extern ptoState PTO;
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

// General
extern lv_obj_t *List;
extern lv_obj_t *autonBtn;
extern lv_obj_t *sensorsBtn;
extern lv_obj_t *motorsBtn;
extern lv_obj_t *settingsBtn;

// Auton Screen
extern lv_obj_t *autonPG;
extern lv_obj_t *summary;
extern lv_obj_t *summaryTxt;
extern lv_obj_t *Left;
extern lv_obj_t *Right;
extern lv_obj_t *solowp;
extern lv_obj_t *skills;

// Motors Screen
extern lv_obj_t *motorsPG;
extern lv_obj_t *Drivemeter;
extern lv_obj_t *Leftfronttemp;
extern lv_obj_t *Leftmidtemp;
extern lv_obj_t *Leftbacktemp;
extern lv_obj_t *Rightfronttemp;
extern lv_obj_t *Rightmidtemp;
extern lv_obj_t *Rightbacktemp;
extern lv_obj_t *catatemp;
extern lv_obj_t *intaketemp;
extern double Leftfronttempval;
extern double Leftmidtempval;
extern double Leftbacktempval;
extern double Rightfronttempval;
extern double Rightmidtempval;
extern double Rightbacktempval;
extern double driveSpdval;

// Sensor Screen
extern lv_obj_t *sensorsPG;
extern lv_obj_t *InertialBox;
extern lv_obj_t *InertialLabelRot;
extern lv_obj_t *InertialLabelHeading;
extern lv_obj_t *InertialLabelPitch;
extern lv_obj_t *InertialLabelRoll;
extern lv_obj_t *InertialLabelYaw;
extern lv_obj_t *InertialLabelAccelX;
extern lv_obj_t *InertialLabelAccelY;
extern lv_obj_t *InertialLabelAccelZ;
extern lv_obj_t *InertialLabel;
extern lv_obj_t *LimitBoxA;
extern lv_obj_t *LimitA;
extern lv_obj_t *LimitALabel;
extern lv_obj_t *LimitBoxB;
extern lv_obj_t *LimitB;
extern lv_obj_t *LimitBLabel;
extern lv_obj_t *cataEncBox;
extern lv_obj_t *cataEncLabel;
extern lv_obj_t *cataEncLabelAngle;
extern lv_obj_t *posEncBox;
extern lv_obj_t *posEncLabel;
extern lv_obj_t *posEncLabelRot;
extern lv_obj_t *OptiBox;
extern lv_obj_t *OptiLabel;

// Settings Screen
extern lv_obj_t *settingsPG;
extern lv_obj_t *grafanaBtn;
extern lv_obj_t *autonRecorderBtn;
extern lv_obj_t *tensorflowliteBtn;
extern lv_obj_t *odomDebugBtn;
extern lv_obj_t *graphingBtn;
extern lv_obj_t *rosBtn;

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