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
			pushNewLinear({{3.35, 4.75}, {3.3, 5.2}});

			// Score 1 ring
			pushNewLinear({{5.2, 4.8}});

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
		grabGoalAt(3.95, 4.2, 0.1);
		setArmStage(0);

		// Re-enable filter
		setIntakeFilterEnabled(1);

		// Score 2 rings
		setIntakeState(1);
		runFollowLinearYield();

		// Score 1 ring
		turnToFace_tiles(5.2, 4.8, false, 60);
		runFollowLinearYield();

		// Sweep corner
		turnToAngle(20, -halfRobotLengthIn * 0.5);
		setSwing2State(1);
		setIntakeState(0);
		driveAndTurnDistanceTiles(1.0, 0.0);
		turnToAngle(-120);

		// Touch ladder
		setIntakeState(1);
		runFollowLinearYield();
	}
}
