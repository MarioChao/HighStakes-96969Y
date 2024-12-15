#include "Autonomous/autonPaths.h"

namespace {
	using namespace autonpaths;
	using namespace autonpaths::pathbuild;

	void loadPaths(int section);

	void doAuton();
}

/// @brief Run the red solo AWP.
void autonpaths::runRedSoloAWP() {
	/* Pre auton */

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

			// Grab goal
			pushNewLinear({{2, 4}}, true, 80);

			// Score 2 rings
			pushNewLinear({{2.65, 4.5}, {2.65, 5.5}}, false, 100);

			// Store 1 ring
			pushNewLinear({{2.37, 4.3}}, true);
			pushNewLinear({{1.9, 5}});

			// Release goal
			pushNewLinear({{0.5, 1}}, true);

			// Grab goal
			pushNewLinear({{0.75, 1.7}});
			pushNewLinear({{2, 2}}, true, 80);

			// Score 2 ring
			pushNewLinear({{2, 1}});

			// Touch ladder
			pushNewLinear({{2.45, 2.47}}, true);
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
		runFollowLinearYield();
		setGoalClampState(1);
		wait(50, msec);

		// Re-eable filter
		setIntakeFilterEnabled(1);

		// Score 2 rings
		setIntakeState(1);
		runFollowLinearYield();

		// Store 1 ring
		runFollowLinearYield();
		setIntakeStoreRing(1);
		runFollowLinearYield();

		// Release goal and push to corner
		setGoalClampState(0);
		runFollowLinearYield();

		// Grab goal
		setIntakeStoreRing(0);
		setIntakeState(0, 0.2);
		runFollowLinearYield();
		runFollowLinearYield();
		setGoalClampState(1);
		wait(50, msec);

		// Score 2 rings
		setIntakeState(1);
		runFollowLinearYield();

		// Touch ladder
		runFollowLinearYield();

		waitUntil(_autonTimer.value() > 14.5);
		setIntakeState(0);
	}
}
