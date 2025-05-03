#pragma once

#include "Autonomous/autonValues.h"
#include "Aespa-Lib/Karina-Data-Structures/linegular.h"
#include "Aespa-Lib/Winter-Utilities/units.h"

#include "main.h"


// Namespace

namespace autonfunctions {


/* General */

void setRobotLookPose(aespa_lib::datas::Linegular pose);
void setRobotPosition(double x_tiles, double y_tiles);
void setRobotRotation(double fieldAngle_degrees);

extern timer _autonTimer;

void setDifferentialUseRelativeRotation(bool useRelativeRotation);

void runLinearPIDPath(std::vector<std::vector<aespa_lib::units::Length>> waypoints, double maxVelocity, bool isReverse = false);


/* Main mechanics */

// Intake
void setIntakeState(int state, double delaySec = 0);
void setIntakeTopState(int, double = 0);
void setIntakeBottomState(int, double = 0);

void setIntakeAntiJam(bool state);

void setIntakeToArm(int, double = 0);
void setIntakeStoreRing(int, double = 0);

void setIntakeFilterOutColor(std::string colorText);
void setIntakeFilterEnabled(bool, double = 0);

// Clamp
void setGoalClampState(bool state, double delaySec = 0);
void setIntakeLiftState(bool state);

// Arm
void setArmHangState(int, double = 0);
void setArmStage(int stageId, double delay_sec = 0, double maxSpeed_pct = 100);

bool isArmResetted();
void setArmResetDefaultStage(int);

// Swing
void setSwingState_left(int, double = 0);
void setSwing2State_left(int, double = 0);
void setSwingState_right(int, double = 0);
void setSwing2State_right(int, double = 0);


/* Legacy wings */

void setFrontWingsState(bool state, double delaySec = 0);
void setLeftWingState(bool state, double delaySec = 0);
void setRightWingState(bool state, double delaySec = 0);
void setBackWingsState(bool state, double delaySec = 0);


}
