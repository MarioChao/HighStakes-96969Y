#include "Autonomous/autonPaths.h"

namespace {
	using namespace autonpaths;
	using namespace autonpaths::pathbuild;

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
	mainOdometry.setPosition(0.79, 3.73);
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
			pushNewLinear({{0.79, 3}});

			// Score on wall stake
			pushNewLinear({{-1, 3}});

			// Score 2 rings
			pushNewLinear({{2.6, 4.7}, {2.67, 5.2}}, false, 100);

			// Score 1 ring
			pushNewLinear({{2.37, 4.3}}, true);
			pushNewLinear({{2, 5}});

			// Touch ladder
			pushNewLinear({{2.48, 3.65}});
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
		turnToFace(2, 4, true);
		async_driveTurnToFace_tiles(2, 4, true, 60);
		waitUntil(_linearPathDistanceError < 0.3);
		setGoalClampState(1);
		waitUntil(_isDriveTurnSettled);
		setArmStage(0);

		// Re-eable filter
		setIntakeFilterEnabled(1);

		// Score 2 rings
		setIntakeState(1);
		runFollowLinearYield();

		// Score 1 ring
		runFollowLinearYield();
		runFollowLinearYield();

		// Touch ladder
		runFollowLinearYield();
		setArmStage(2); // todo: zip ties?

		waitUntil(_autonTimer.value() > 14.5);
		setIntakeState(0);
	}
}
