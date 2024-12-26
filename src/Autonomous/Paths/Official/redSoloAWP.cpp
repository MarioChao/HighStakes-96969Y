#include "Autonomous/autonPaths.h"

namespace {
	using namespace autonpaths;
	using namespace autonpaths::pathbuild;
	using namespace autonpaths::combination;

	void loadPaths(int section);

	void doAuton1();
	void doAuton2();
}

/// @brief Run the red solo AWP.
void autonpaths::runRedSoloAWP() {
	/* Pre auton */

	// Timer
	_autonTimer.reset();

	// Set position and rotation
	mainOdometry.printDebug();
	mainOdometry.setPosition(0.8, 3.73);
	setRobotRotation(-180);
	mainOdometry.printDebug();

	// Set config
	setDifferentialUseRelativeRotation(true);

	// Wait for arm reset
	// waitUntil(isArmResetted());
	setArmResetDefaultStage(2);


	/* Auton */
	loadPaths(1);
	doAuton1();

	loadPaths(2);
	doAuton2();
}

namespace {
	void loadPaths(int section) {
		// Clear
		clearLinear();
		clearSplines();

		if (section == 1) {
			// Go to alliance wall stake
			pushNewLinear({{0.8, 3}});

			// Score on wall stake
			pushNewLinear({{-1, 3}}, false, autonvals::scoreAllianceWallStakeVelocity_pct);
		} else if (section == 2) {
			// Score 1 ring
			// pushNewLinear({{2.67, 4.72}}, false, 80);

			// Score 1 ring
			pushNewLinear({{2, 5}});

			// Store middle
			pushNewLinear({{0.9, 2.55}});

			// Score 1 ring
			pushNewLinear({{2, 1.2}});

			// Touch ladder
			pushNewLinear({{2.5, 2.4}}, false, 60);
		}
	}

	void doAuton1() {
		// Intake filter at hood
		setIntakeFilterEnabled(0);
		setIntakeStoreRing(1);

		// Score on alliance wall stake
		runFollowLinearYield();
		setIntakeStoreRing(0, 0.5);
		wait(50, msec);
		runFollowLinearYield();
		setIntakeState(-1, 0.1);
		driveDistanceTiles(-0.5);

		// Re-enable filter
		setIntakeFilterEnabled(1, 1.0);
	}

	void doAuton2() {
		// Grab goal
		grabGoalAt(6 - (3.95), 4.15);

		// Score
		setIntakeState(1);
		// runFollowLinearYield();
		// driveDistanceTiles(-0.5);

		// Score
		runFollowLinearYield();

		// Store middle
		setIntakeStoreRing(1, 1.5);
		setGoalClampState(0, 1.5);
		runFollowLinearYield();

		// Grab goal
		grabGoalAt(6 - (3.8), 1.95);

		// Score
		setIntakeStoreRing(0);
		setIntakeState(1);
		runFollowLinearYield();

		// Touch ladder
		setArmStage(4);
		runFollowLinearYield();
	}
}
