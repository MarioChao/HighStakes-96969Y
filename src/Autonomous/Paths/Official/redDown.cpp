#include "Autonomous/autonPaths.h"

namespace {
	using namespace autonpaths;
	using namespace autonpaths::pathbuild;

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
	mainOdometry.setPosition(0.6, 0.35);
	setRobotRotation(66.56);
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
			// Grab rushed goal
			pushNewLinear({{2.1, 1.02}}, true, 60);

			// Grab goal
			pushNewLinear({{2, 2}}, true, 60);

			// Score 1 ring
			pushNewLinear({{0.64, 0}});

			// Score alliance wall stake
			pushNewLinear({{0.55, 3.}}, true);
			pushNewLinear({{0, 3}});

			// Touch ladder
			pushNewLinear({{2.11, 2.45}}, true);
		}
	}

	void doAuton() {
		/* Rush middle goal */

		// Store + rush
		setIntakeStoreRing(1);
		async_driveTurnToFace_tiles(2.15, 1.0);
		// driveTurnToFace_tiles(2.15, 1.0, false, 80);

		// Deploy
		waitUntil(_linearPathDistanceError < 0.2);
		setSwing2State(1);

		// Go back & un-deploy
		waitUntil(_isDriveTurnSettled);
		setSwing2State(0, 0.4);
		driveDistanceTiles(-0.5);

		// Grab goal
		runFollowLinearYield();
		setGoalClampState(1);

		// Score stored
		setIntakeStoreRing(0);
		setIntakeState(1);
		wait(200, msec);

		// Release goal
		setGoalClampState(0);

		// Grab goal
		setIntakeState(0);
		driveDistanceTiles(0.5);
		runFollowLinearYield();
		setGoalClampState(1);
		wait(200, msec);

		// Go to corner
		async_driveTurnToFace_tiles(0.6, 0.5);

		// Deploy
		waitUntil(_linearPathDistanceError < 0.2);
		setSwingState(1);

		// Swing out rings
		waitUntil(_isDriveTurnSettled);
		turnToAngleVelocity(160, 50);
		setSwingState(0);

		// Score 1 ring
		setIntakeState(1);
		runFollowLinearYield();

		// Score alliance wall stake
		setArmStage(2, 0.5);
		runFollowLinearYield();
		setIntakeState(0);
		runFollowLinearYield();
		driveDistanceTiles(-0.5);

		// Touch ladder
		setArmStage(0, 1.0);

		// Follow path
		runFollowLinearYield();
	}
}
