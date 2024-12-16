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
	mainOdometry.setPosition(0.6, 0.45);
	setRobotRotation(70);
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
			pushNewLinear({{0.6, 1.1}});

			// Store corner
			pushNewLinear({{1.8, 0.8}});

			// Score on wall stake
			pushNewLinear({{-0.3, 3}}, false, 75);

			// Touch ladder
			pushNewLinear({{2.15, 2.3}}, true);
		}
	}

	void doAuton() {
		// Store ring + rush goal
		setIntakeStoreRing(1);
		async_driveTurnToFace_tiles(2.15, 1.0);

		// Deploy
		waitUntil(_linearPathDistanceError < 0.15);
		setSwing2State(1);
		setSwing2State(0, 0.6);

		// Go back & un-deploy
		waitUntil(_isDriveTurnSettled);
		driveDistanceTiles(-0.7);
		setSwing2State(0);

		// Grab rushed goal
		setIntakeStoreRing(0);
		grabGoalAt(2.4, 0.8);

		// Score stored
		setIntakeState(1);

		// Sweep corner
		setArmStage(2);
		runFollowLinearYield();
		turnToAngle(-160);
		setSwing2State(1);
		driveAndTurnDistanceTiles(1.0, -180.0);

		// Store corner
		setIntakeStoreRing(1);
		setGoalClampState(0, 0.3);
		runFollowLinearYield();
		setSwing2State(0);

		// Grab goal
		setIntakeStoreRing(0);
		grabGoalAt(2, 1.9);

		// Score stored
		setIntakeState(1);

		// Score alliance wall stake
		runFollowLinearYield();
		driveDistanceTiles(-0.4);

		// Touch ladder
		setArmStage(0, 0.5);
		runFollowLinearYield();
	}
}
