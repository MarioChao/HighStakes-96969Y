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
	mainOdometry.setPosition(0.7, 3.73);
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

			// Grab goal
			pushNewLinear({{2, 4}}, true, 60);

			// Score 2 rings
			pushNewLinear({{2.6, 4.6}, {2.65, 5.2}}, false, 60);

			// Score 1 ring
			pushNewLinear({{2.37, 4.3}}, true);
			pushNewLinear({{2, 5}});

			// Store 1 ring
			pushNewLinear({{1.19, 3.43}});
			pushNewLinear({{0.88, 2.5}}, false, 50);

			// Grab goal
			pushNewLinear({{2, 2}}, true, 60);

			// Score 2 ring
			pushNewLinear({{2, 1}});

			// Touch ladder
			pushNewLinear({{2.45, 2.47}}, true);
		}
	}

	void doAuton() {
		// Intake filter at hood
		setIntakeFilterEnabled(0);
		setIntakeState(1);

		// Score on alliance wall stake
		runFollowLinearYield();
		setIntakeState(0, 0.4);
		wait(50, msec);
		runFollowLinearYield();
		driveDistanceTiles(-0.5);

		// Re-eable filter
		setIntakeFilterEnabled(1);

		// Grab goal
		runFollowLinearYield();
		setGoalClampState(1);
		wait(200, msec);

		// Score 2 rings
		setIntakeState(1);
		runFollowLinearYield();

		// Score 1 ring
		runFollowLinearYield();		
		runFollowLinearYield();

		// Store 1 ring
		runFollowLinearYield();
		setGoalClampState(0);
		setIntakeStoreRing(1);
		runFollowLinearYield();

		// Grab goal
		setIntakeStoreRing(0, 0.4);
		setIntakeState(0, 0.5);
		runFollowLinearYield();
		setGoalClampState(1);
		wait(200, msec);

		// Score 2 rings
		setIntakeState(1);
		runFollowLinearYield();

		// Touch ladder
		runFollowLinearYield();

		waitUntil(_autonTimer.value() > 14.5);
		setIntakeState(0);
	}
}
