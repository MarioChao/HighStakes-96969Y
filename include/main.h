#pragma once

#include "vex.h"
#include <iostream>
#include <vector>

// Forward declaration
class Odometry;
class RobotSimulator;

// Global variables

// Competition instance
extern competition Competition;

// Drive info
extern double motSpeedRpm, motAimSpeedRpm;

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
