#include "Autonomous/autonPaths.h"

namespace {
	using namespace autonpaths;
	using namespace autonpaths::pathbuild;

	void loadPaths(int section);

	void doAuton();
}

/// @brief Run the solo AWP.
void autonpaths::runSoloAWP() {
	/* Pre auton */

	// Timer
	_autonTimer.reset();

	// Set position and rotation
	mainOdometry.printDebug();
	mainOdometry.setPosition(0.7, 3.7);
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
			// Go to alliance wall stake
			pushNewLinear({{0.7, 3}});

			// Score on wall stake
			pushNewLinear({{-1, 3}});
		}
	}

	void doAuton() {
		// Score on alliance wall stake
		setIntakeState(1);
		runFollowLinearYield();
		setIntakeState(0, 0.4);
		runFollowLinearYield();

		// Grab goal
	}
}
