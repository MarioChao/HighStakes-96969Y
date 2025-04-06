#pragma once

#include "main.h"
#include "Pas1-Lib/Planning/Trajectories/trajectory-planner.h"
#include "Pas1-Lib/Planning/Profiles/spline-profile.h"
#include "Aespa-Lib/Karina-Data-Structures/named-storage.h"
#include "Cosmetics/Simulation/robotSimulator.h"


// ---------- Global variables ----------

// Competition instance
extern competition Competition;

// Intake info
extern int intakePartType;

// Arm info
extern bool isArmPneumatic;

// Timer
extern timer drivingTimer;

// Simulator
extern RobotSimulator robotSimulator;
extern bool mainUseSimulator;

// Trajectory
extern pas1_lib::planning::trajectories::TrajectoryPlanner testTrajectoryPlan;
extern timer trajectoryTestTimer;

// Spline Profile Storage
extern aespa_lib::datas::NamedStorage<pas1_lib::planning::profiles::SplineProfile> splineProfile_storage;
