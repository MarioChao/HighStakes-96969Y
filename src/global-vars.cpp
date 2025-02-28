#include "global-vars.h"

// ---------- Global variables ----------

competition Competition;

int intakePartType = 1;

bool isArmPneumatic = false;

timer drivingTimer;

chassis_tracker::Odometry mainOdometry;

RobotSimulator robotSimulator;
bool mainUseSimulator = false;

TrajectoryPlanner testTrajectoryPlan;
timer trajectoryTestTimer;
