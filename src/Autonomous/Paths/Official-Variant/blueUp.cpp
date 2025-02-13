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
			pushNewLinear({{5.2, 3}});

			// Score on wall stake
			pushNewLinear({{7, 3}}, false, autonvals::scoreAllianceWallStakeVelocity_pct);

			// Score 2 rings
			pushNewLinear({{3.27, 4.72}}, false, 40);
			// pushNewLinear({{3.3, 5.3}}, false, 40);

			// Score 1 ring
			pushNewLinear({{4.2, 5.2}});

			// Go to corner
			pushNewLinear({{5.2, 4.6}});

			// Touch ladder
			pushNewLinear({{4.25, 3.35}});
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
		driveDistanceTiles(-0.4);

		// Grab goal
		setIntakeState(-1);
		grabGoalAt(3.95, 4.1);
		setArmStage(0);

		// Re-enable filter
		setIntakeFilterEnabled(1);

		// Score 2 rings
		setIntakeState(1);
		// setSwingState(1);
		runFollowLinearYield();
		// runFollowLinearYield();
		// setSwingState(0);

		// Score 1 ring
		turnToFace_tiles(4.2, 5.2, false, 60);
		runFollowLinearYield();

		// Go to corner
		runFollowLinearYield();

		// Sweep corner
		turnToAngle(20, -halfRobotLengthIn * 0.5);
		setSwing2State(1);
		turnToAngle(40);
		setIntakeState(0);
		driveAndTurnDistanceTiles(1.0, 0.0);
		turnToAngle(-120);

		// Touch ladder
		setSwing2State(0);
		setIntakeState(1);
		runFollowLinearYield();
	}
}
