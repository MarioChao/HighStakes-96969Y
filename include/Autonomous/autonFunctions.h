#pragma once

#include "Autonomous/autonValues.h"

#include "main.h"


// Forward declaration

namespace pas1_lib {
namespace planning {
namespace splines {
class SplineCurve;
class CurveSampler;
}
namespace trajectories {
class TrajectoryPlanner;
}
}
}


// Namespace

namespace autonfunctions {


/* General */

void setRobotPosition(double x_tiles, double y_tiles);
void setRobotRotation(double fieldAngle_degrees);

extern timer _autonTimer;

void setDifferentialUseRelativeRotation(bool useRelativeRotation);

void runLinearPIDPath(std::vector<std::vector<double>> waypoints, double maxVelocity, bool isReverse = false);


/* Main mechanics */

// Intake
void setIntakeState(int state, double delaySec = 0);
void setIntakeTopState(int, double = 0);
void setIntakeBottomState(int, double = 0);

void setIntakeToArm(int, double = 0);
void setIntakeStoreRing(int, double = 0);

void setIntakeFilterOutColor(std::string colorText);
void setIntakeFilterEnabled(bool, double = 0);

// Clamp
void setGoalClampState(bool state, double delaySec = 0);
void setIntakeLiftState(bool state);

// Arm
void setArmHangState(int, double = 0);
void setArmStage(int, double = 0);

bool isArmResetted();
void setArmResetDefaultStage(int);

// Swing
void setSwingState(int, double = 0);
void setSwing2State(int, double = 0);


/* Legacy wings */

void setFrontWingsState(bool state, double delaySec = 0);
void setLeftWingState(bool state, double delaySec = 0);
void setRightWingState(bool state, double delaySec = 0);
void setBackWingsState(bool state, double delaySec = 0);


}
