#pragma once

#include "main.h"
#include "Pas1-Lib/Chassis-Tracker/odometry.h"
#include "Simulation/robotSimulator.h"
#include "Pas1-Lib/Planning/Trajectories/trajectoryPlanner.h"

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
extern chassis_tracker::Odometry mainOdometry;

// Simulator
extern RobotSimulator robotSimulator;
extern bool mainUseSimulator;

// Trajectory
extern TrajectoryPlanner testTrajectoryPlan;
extern timer trajectoryTestTimer;
