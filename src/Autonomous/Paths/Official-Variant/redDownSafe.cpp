#include "Autonomous/autonPaths.h"

namespace {
using namespace autonpaths;
using namespace autonpaths::pathbuild;
using namespace autonpaths::combination;

void loadPaths(int section);

void doAuton();
}

/// @brief Run the 15-seconds new red-down autonomous.
void autonpaths::runAutonRedDownSafe() {
	/* Pre skills */

	// Timer
	_autonTimer.reset();

	// Set config
	setRobotPosition(1.0, 1.5);
	setRobotRotation(112.25);
	// setDifferentialUseRelativeRotation(true);

	// Wait for arm reset
	waitUntil(isArmResetted());


	/* Auton */
	doAuton();
}

namespace {
void loadPaths(int section) {
	// Clear
	clearLinear();

	if (section == 1) {
	}
}

void doAuton() {
}
}
