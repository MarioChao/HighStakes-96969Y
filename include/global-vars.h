#pragma once

#include "main.h"
#include "ChassisTracker/odometry.h"
#include "Simulation/robotSimulator.h"
#include "GraphUtilities/trajectoryPlanner.h"

// Forward declaration

// namespace chassis_tracker {
// 	class Odometry;
// }
// class RobotSimulator;
// class TrajectoryPlanner;


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
