#pragma once

#include "vex.h"
#include <iostream>
#include <vector>

// Forward declaration
class Odometry;

// Global variables

// Competition instance
extern competition Competition;

// Drive info
extern double motSpeedRpm, motAimSpeedRpm;

// Video info
extern int playingVideoId;

// Timer
extern timer drivingTimer;

// Odometry
extern Odometry mainOdometry;
