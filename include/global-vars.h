#pragma once

#include "main.h"
#include "Pas1-Lib/Chassis-Tracker/odometry.h"
#include "Pas1-Lib/Planning/Trajectories/trajectoryPlanner_old.h"
#include "Simulation/robotSimulator.h"


// ---------- Global variables ----------

// Competition instance
extern competition Competition;

// Intake info
extern int intakePartType;

// Arm info
extern bool isArmPneumatic;

// Timer
extern timer drivingTimer;

// Odometry
extern pas1_lib::chassis_tracker::Odometry mainOdometry;

// Simulator
extern RobotSimulator robotSimulator;
extern bool mainUseSimulator;

// Trajectory
extern TrajectoryPlanner_Old testTrajectoryPlan;
extern timer trajectoryTestTimer;
