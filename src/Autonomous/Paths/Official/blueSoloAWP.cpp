#include "Autonomous/autonPaths.h"

namespace {
	using namespace autonpaths;
	using namespace autonpaths::pathbuild;
	using namespace autonpaths::combination;

	void loadPaths(int section);

	void doAuton1();
	void doAuton2();
}

/// @brief Run the blue solo AWP.
void autonpaths::runBlueSoloAWP() {
	/* Pre auton */

	// Timer
	_autonTimer.reset();

	// Set position and rotation
	mainOdometry.printDebug();
	mainOdometry.setPosition(5.2, 3.73);
	setRobotRotation(180);
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
			pushNewLinear({{5.2, 3}});

			// Score on wall stake
			pushNewLinear({{7, 3}}, false, autonvals::scoreAllianceWallStakeVelocity_pct);
		} else if (section == 2) {
			// Score 1 ring
			// pushNewLinear({{3.33, 4.72}}, false, 80);

			// Score 1 ring
			pushNewLinear({{4, 4.9}});

			// Store middle
			pushNewLinear({{5.11, 2.55}});

			// Score 1 ring
			pushNewLinear({{4, 1.2}});

			// Touch ladder
			pushNewLinear({{3.5, 2.4}}, false, 60);
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
		grabGoalAt(3.95, 4.15);

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
		grabGoalAt(3.8, 1.95);

		// Score
		setIntakeStoreRing(0);
		setIntakeState(1);
		runFollowLinearYield();

		// Touch ladder
		setArmStage(4);
		runFollowLinearYield();
	}
}
