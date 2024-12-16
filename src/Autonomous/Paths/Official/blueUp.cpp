#include "Autonomous/autonPaths.h"

namespace {
	using namespace autonpaths;
	using namespace autonpaths::pathbuild;
	using namespace autonpaths::combination;

	void loadPaths(int section);

	void doAuton();
}

/// @brief Run the 15-seconds new blue-up autonomous.
void autonpaths::runAutonBlueUp() {
	/* Pre skills */

	// Timer
	_autonTimer.reset();

	// Set position and rotation
	mainOdometry.printDebug();
	mainOdometry.setPosition(6 - 0.79, 3.78);
	setRobotRotation(180);
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
			pushNewLinear({{6 - (0.79), 3}});

			// Score on wall stake
			pushNewLinear({{6 - (-1), 3}}, false, autonvals::scoreWallStakeVelocity_pct);

			// Score 2 rings
			pushNewLinear({{6 - (2.6), 4.75}, {6 - (2.67), 5.2}});

			// Score 1 ring
			pushNewLinear({{6 - (2), 4.8}});

			// Sweep corner
			pushNewLinear({{6 - (0.25), 5.75}});

			// Touch ladder
			pushNewLinear({{6 - (2), 3}});
		}
	}

	void doAuton() {
		// Intake filter at hood
		setIntakeFilterEnabled(0);
		setIntakeStoreRing(1);

		// Score on alliance wall stake
		runFollowLinearYield();
		setIntakeStoreRing(0, 0.5);
		wait(50, msec);
		runFollowLinearYield();
		driveDistanceTiles(-0.5);

		// Grab goal
		setIntakeState(-1);
		turnToFace_tiles(6 - (2), 4, true);
		async_driveTurnToFace_tiles(6 - (2), 4, true, 60);
		waitUntil(_linearPathDistanceError < 0.3);
		setGoalClampState(1);
		waitUntil(_isDriveTurnSettled);
		setArmStage(0);

		// Re-enable filter
		setIntakeFilterEnabled(1);

		// Score 2 rings
		setIntakeState(1);
		runFollowLinearYield();

		// Score 1 ring
		turnToAngleVelocity(-(-90), 60);
		runFollowLinearYield();

		// Score corner
		setSwing2State(1);
		setIntakeState(0, 0.5);
		runFollowLinearYield();

		// Touch ladder
		setIntakeState(1);
		runFollowLinearYield();
	}
}
