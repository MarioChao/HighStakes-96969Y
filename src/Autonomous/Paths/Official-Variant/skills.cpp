#include "Autonomous/autonPaths.h"

namespace {
using namespace autonpaths;
using namespace autonpaths::pathbuild;
using namespace autonpaths::combination;

void loadPaths(int section);

void doAuton();
}

/// @brief Run the autonomous skills.
void autonpaths::runAutonSkills() {
	/* Pre skills */

	// Timer
	_autonTimer.reset();

	// Set position and rotation
	mainOdometry.setPosition_scaled(0.792, 3);
	setRobotRotation(-90.0);

	// Set config
	setDifferentialUseRelativeRotation(true);

	// Wait for arm reset
	waitUntil(isArmResetted());


	/* Skills */
	loadPaths(1);
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
