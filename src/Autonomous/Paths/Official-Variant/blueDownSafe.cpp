#include "Autonomous/autonPaths.h"

namespace {
using namespace autonpaths;
using namespace autonpaths::pathbuild;
using namespace autonpaths::combination;

void loadPaths(int section);

void doAuton();
}

/// @brief Run the 15-seconds new blue-down autonomous.
void autonpaths::runAutonBlueDownSafe() {
	/* Pre skills */

	// Timer
	_autonTimer.reset();

	// Set config
	setRobotPosition(5.43, 1.53);
	setRobotRotation(-111.7);
	setDifferentialUseRelativeRotation(true);

	// Wait for arm reset
	waitUntil(isArmResetted());


	/* Auton */
	loadPaths(1);
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
