#pragma once

#include "vex.h"
#include <iostream>
#include <vector>
#include <cmath>

// Forward declaration
class Odometry;
class RobotSimulator;
class TrajectoryPlanner;

// Global variables

// Competition instance
extern competition Competition;

// Intake info
extern int intakePart;

// Arm info
extern bool isArmPneumatic;

// Timer
extern timer drivingTimer;

// Odometry
extern Odometry mainOdometry;

// Simulator
extern RobotSimulator robotSimulator;
extern bool mainUseSimulator;

// Trajectory
extern TrajectoryPlanner testTrajectoryPlan;
extern timer trajectoryTestTimer;
