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

	// Set position and rotation
	mainOdometry.printDebug();
	mainOdometry.setPosition(5.43, 1.53);
	setRobotRotation(-111.7);
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
			pushNewLinear({{5.7, 0.2}});

			// Store corner
			pushNewLinear({{4.7, 1.4}});

			// Score preload
			pushNewLinear({{5.8, 2.3}});

			// Touch ladder
			pushNewLinear({{3.45, 2.5}}, false, 60);
		}
	}

	void doAuton() {
		// Store ring + rush goal
		setIntakeStoreRing(1);
		async_driveTurnToFace_tiles(3.78, 0.93);

		// Deploy
		waitUntil(_linearPathDistanceError < 0.15);
		setSwing2State(1);
		turnToAngle(-111.7);
		wait(autonvals::rushGoalDeployDelay_msec, msec);

		// Go back & un-deploy
		waitUntil(_isDriveTurnSettled);
		setSwing2State(0, 0.8);
		driveTurnToFace_tiles(4.76, 1.41, true, 60);
		setSwing2State(0);

		// Grab rushed goal
		setIntakeStoreRing(0);
		grabGoalAt(3.6, 1.1);

		// Score stored
		setIntakeState(1);

		// Sweep corner
		setArmStage(2);
		setSwing2State(1);
		runFollowLinearYield();

		// Store corner
		setIntakeStoreRing(1);
		setGoalClampState(0, 1.0);
		runFollowLinearYield();
		turnToAngle(-180);
		setSwing2State(0);

		// Grab goal
		setIntakeStoreRing(0);
		grabGoalAt(3.9, 2);

		// Score stored
		setIntakeState(1);

		// Score preload
		setArmStage(4);
		runFollowLinearYield();

		// Touch ladder
		runFollowLinearYield();
	}
}
