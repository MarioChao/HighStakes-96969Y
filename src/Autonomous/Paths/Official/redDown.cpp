#include "Autonomous/autonPaths.h"

namespace {
	using namespace autonpaths;
	using namespace autonpaths::pathbuild;
	using namespace autonpaths::combination;

	void loadPaths(int section);

	void doAuton();
}

/// @brief Run the 15-seconds new red-down autonomous.
void autonpaths::runAutonRedDown() {
	/* Pre skills */

	// Timer
	_autonTimer.reset();

	// Set position and rotation
	mainOdometry.printDebug();
	mainOdometry.setPosition(0.6, 0.5);
	setRobotRotation(73);
	mainOdometry.printDebug();

	// Set config
	setDifferentialUseRelativeRotation(true);

	// Wait for arm reset
	// waitUntil(isArmResetted());


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
			// Sweep corner
			pushNewLinear({{0.3, 1.1}});
			pushNewLinear({{0, 0.2}});

			// Store corner
			pushNewLinear({{2, 1}});

			// Score on wall stake
			pushNewLinear({{0, 3}}, false, 75);

			// Touch ladder
			pushNewLinear({{2.15, 2.35}}, true);
		}
	}

	void doAuton() {
		// Store ring + rush goal
		setIntakeStoreRing(1);
		async_driveTurnToFace_tiles(2.25, 1.0);

		// Deploy
		waitUntil(_linearPathDistanceError < 0.1);
		setSwing2State(1);
		setSwing2State(0, 0.6);

		// Go back & un-deploy
		waitUntil(_isDriveTurnSettled);
		driveDistanceTiles(-0.5);
		setSwing2State(0);

		// Grab rushed goal
		setIntakeStoreRing(0);
		grabGoalAt(2.5, 0.8);

		// Score stored
		setIntakeState(1);

		// Sweep corner
		setArmStage(2);
		runFollowLinearYield();
		setSwing2State(1, 0.3);
		runFollowLinearYield();

		// Store corner
		setSwing2State(0);
		setIntakeStoreRing(1);
		setGoalClampState(0, 0.3);
		runFollowLinearYield();

		// Grab goal
		setIntakeStoreRing(0);
		grabGoalAt(2, 2);

		// Score stored
		setIntakeState(1);

		// Score alliance wall stake
		runFollowLinearYield();
		driveDistanceTiles(-0.2);

		// Touch ladder
		setArmStage(0, 0.5);
		runFollowLinearYield();
	}
}
