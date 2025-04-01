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

	// Set position and rotation
	mainOdometry.printDebug();
	mainOdometry.setPosition_scaled(0.77, 3.78);
	setRobotRotation(-180);
	mainOdometry.printDebug();

	// Set config
	setDifferentialUseRelativeRotation(true);

	// Wait for arm reset
	// waitUntil(isArmResetted());
	setArmResetDefaultStage(2);


	/* Auton */
	loadPaths(1);
	doAuton();
}

namespace {
void loadPaths(int section) {
	// Clear
	clearLinear();
	clearSplines();

	if (section == 1) {

	}
}

void doAuton() {

}
}
