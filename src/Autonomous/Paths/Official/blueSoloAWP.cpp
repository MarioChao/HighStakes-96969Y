#include "Autonomous/autonPaths.h"

namespace {
using namespace autonpaths;
using namespace autonpaths::pathbuild;
using namespace autonpaths::combination;

void loadPaths(int section);

void doAuton1();
}

/// @brief Run the blue solo AWP.
void autonpaths::runBlueSoloAWP() {
	/* Pre auton */

	// Timer
	_autonTimer.reset();

	// Set config
	setRobotPosition(5.2, 3.73);
	setRobotRotation(180);
	setDifferentialUseRelativeRotation(true);

	// Wait for arm reset
	waitUntil(isArmResetted());


	/* Auton */
	loadPaths(1);
	doAuton1();
}

namespace {
void loadPaths(int section) {
	// Clear
	clearLinear();

	if (section == 1) {

	}
}

void doAuton1() {

}
}
