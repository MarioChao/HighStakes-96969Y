#include "Autonomous/autonPaths.h"

namespace {
using namespace autonpaths;
using namespace autonpaths::pathbuild;
using namespace autonpaths::combination;

void loadPaths(int section);

void doAuton();
}

/// @brief Run the 15-seconds new red-up autonomous.
void autonpaths::runAutonRedUp() {
	/* Pre skills */

	// Timer
	_autonTimer.reset();

	// Set config
	setRobotPosition(0.77, 3.78);
	setRobotRotation(-180);
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
