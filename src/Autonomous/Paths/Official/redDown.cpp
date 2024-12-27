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
	mainOdometry.setPosition(0.56, 0.45);
	setRobotRotation(68);
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
			// Go to corner
			pushNewLinear({{0.4, 0.7}});

			// Score preload
			// pushNewLinear({{0.4, 0.55}});

			// Store corner
			pushNewLinear({{1.8, 0.8}});

			// Touch ladder
			pushNewLinear({{2, 2.7}});
		}
	}

	void doAuton() {
		// Store ring + rush goal
		setIntakeStoreRing(1);
		async_driveTurnToFace_tiles(2.21, 1.07);  // (2.2, 1)

		// Deploy
		waitUntil(_linearPathDistanceError < 0.15);
		setSwing2State(1);
		turnToAngle(68);
		wait(autonvals::rushGoalDeployDelay_msec, msec);

		// Go back & un-deploy
		waitUntil(_isDriveTurnSettled);
		setSwing2State(0, 0.8);
		driveTurnToFace_tiles(1.24, 0.64, true, 60);
		setSwing2State(0);

		// Grab rushed goal
		setIntakeStoreRing(0);
		grabGoalAt(2.4, 0.8);

		// Score stored
		setIntakeState(1);

		// Go to corner
		// setIntakeFilterEnabled(0);
		setArmStage(2);
		runFollowLinearYield();
		turnToAngle(-160);
		driveDistanceTiles(-0.3);
		setSwing2State(1);
		turnToAngle(-143);

		// Sweep corner
		// driveAndTurnDistanceTiles(1.5, -180.0);
		driveAndTurnDistanceTiles(1.5, -143);
		turnToAngle(90);
		 
		// Store corner
		setIntakeStoreRing(1);
		setSwing2State(0);
		setGoalClampState(0, 0.7);
		runFollowLinearYield();

		// Grab goal
		setIntakeStoreRing(0);
		grabGoalAt(2, 1.9);

		// Score stored
		setIntakeState(1);

		// Touch ladder
		setArmStage(0);
		runFollowLinearYield();
	}
}
