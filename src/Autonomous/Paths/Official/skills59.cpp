#include "Autonomous/autonPaths.h"

namespace {
	using namespace autonpaths;
	using namespace autonpaths::pathbuild;
	using namespace autonpaths::combination;

	void loadPaths(int section);

	void firstCorner();
	void secondCorner();
	void thirdCorner();
	void fourthCorner();
	void finalSkills();
}

/// @brief Run the autonomous skills.
void autonpaths::runAutonSkills59() {
	/* Pre skills */

	// Timer
	_autonTimer.reset();

	// Set position and rotation
	mainOdometry.setPosition(0.792, 3);
	setRobotRotation(-90.0);

	// Set config
	setDifferentialUseRelativeRotation(true);

	// Wait for arm reset
	// waitUntil(isArmResetted());
	setArmResetDefaultStage(2);


	/* Skills */
	loadPaths(1);
	firstCorner();

	loadPaths(2);
	secondCorner();

	loadPaths(3);
	thirdCorner();

	loadPaths(4);
	fourthCorner();

	loadPaths(5);
	finalSkills();
}

namespace {
	void loadPaths(int section) {
		// Clear
		clearLinear();
		clearSplines();

		if (section == 1) {
			// Score on wall stake
			pushNewLinear({{0, 3}}, false, autonvals::scoreAllianceWallStakeVelocity_pct);

			// Redirect 1 ring
			pushNewLinear({{1.93, 2}});

			// Score 2 rings
			pushNewLinear({{2, 1.15}, {3.9, 1.15}});

			// Score on wall stake
			pushNewLinear({{3.0, 1.2}}, true);
			pushNewLinear({{3, 0}}, false, autonvals::scoreNeutralWallStakeVelocity_pct);

			// Score 3 rings
			pushNewLinear({{1, 1.15}, {0.5, 1.15}, {1.35, 0.35}});

			// Place goal at corner
			pushNewLinear({{0.39, 0.39}}, true);
		} else if (section == 2) {
			// Redirect 1 ring
			pushNewLinear({{2, 4}});

			// Score 2 rings
			pushNewLinear({{2, 4.85}, {3.9, 4.85}});

			// Score on wall stake
			pushNewLinear({{3, 4.8}}, true);
			pushNewLinear({{3, 6}}, false, autonvals::scoreNeutralWallStakeVelocity_pct);

			// Score 3 rings
			pushNewLinear({{1, 4.85}, {0.5, 4.85}, {1.35, 5.65}});

			// Place goal at corner
			pushNewLinear({{0.39, 5.61}}, true);
		} else if (section == 3) {
			// Redirect 1 ring
			pushNewLinear({{3, 3}});

			// Store 1 ring
			pushNewLinear({{4, 4}});

			// Score on wall stake
			pushNewLinear({{6, 3}}, false, autonvals::scoreAllianceWallStakeVelocity_pct);

			// Score 2 rings
			pushNewLinear({{4.38, 4.03}});
			pushNewLinear({{5, 5.47}});

			// Score 3 rings
			pushNewLinear({{4.65, 4}, {4.75, 2.8}}, true);
			pushNewLinear({{4, 2}, {4.95, 1.05}, {4.95, 0.55}});

			// Place goal at corner
			pushNewLinear({{6, 0}}, true);

		} else if (section == 4) {
			// Prepare to grab goal
			pushNewLinear({{4.37, 1.96}});

			// Place goal at corner
			pushNewLinear({{6, 6}}, true);
		} else if (section == 5) {
			// Climb on ladder
			pushNewLinear({{3.85, 3.85}});
			pushNewLinear({{3, 3}}, false, 70);
		}
	}

	void firstCorner() {
		// Wall stake
		waitUntil(isArmResetted());
		wait(600, msec);
		runFollowLinearYield();
		driveDistanceTiles(-0.5);

		// Goal
		grabGoalAt(1, 2);

		// Redirect
		setIntakeToArm(1);
		setIntakeState(1);
		runFollowLinearYield();

		// Score
		setIntakeToArm(0, 0.5);
		setArmStage(0, 1.5);
		runFollowLinearYield();

		// Wall stake
		setArmStage(3);
		runFollowLinearYield();
		runFollowLinearYield();
		// Odometry wall align
		mainOdometry.setPosition(mainOdometry.getX(), 0.45);
		driveDistanceTiles(-0.5);

		// Score
		setArmStage(0, 1);
		runFollowLinearYield();

		// Place goal
		runFollowLinearYield();
		setGoalClampState(0);
		driveDistanceTiles(0.5);
	}

	void secondCorner() {
		// Goal
		grabGoalAt(1, 4.1);

		// Redirect
		setIntakeToArm(1);
		setIntakeState(1);
		runFollowLinearYield();

		// Score
		setIntakeToArm(0, 0.5);
		setArmStage(0, 1.5);
		runFollowLinearYield();

		// Wall stake
		setArmStage(3);
		setIntakeState(1);
		runFollowLinearYield();
		runFollowLinearYield();
		// Odometry wall align
		mainOdometry.setPosition(mainOdometry.getX(), 5.55);
		driveDistanceTiles(-0.5);

		// Score
		setArmStage(0, 1);
		runFollowLinearYield();

		// Place goal
		runFollowLinearYield();
		setGoalClampState(0);
		driveDistanceTiles(0.5);
	}

	void thirdCorner() {
		// Redirect
		setIntakeToArm(1, 0.5);
		setIntakeState(1);
		runFollowLinearYield();

		// Store
		setIntakeToArm(0, 0.5);
		setIntakeStoreRing(1, 0.5);
		runFollowLinearYield();

		// Goal
		grabGoalAt(5, 3);

		// Wall stake
		setArmStage(2);
		runFollowLinearYield();
		driveDistanceTiles(-0.5);
		setIntakeState(1);
		setArmStage(0, 1.0);

		// Score 2 rings
		runFollowLinearYield();
		setSwing2State(1, 0.6);
		runFollowLinearYield();
		setSwing2State(0);

		// Score 3 rings
		runFollowLinearYield();
		runFollowLinearYield();

		// Place goal
		runFollowLinearYield();
		driveDistanceTiles(0.5);
	}

	void fourthCorner() {
		// Fake grab goal
		runFollowLinearYield();
		setGoalClampState(1);
		grabGoalAt(5.46, 3.95);

		// Place goal
		runFollowLinearYield();
		setGoalClampState(0);
		driveDistanceTiles(0.5);
	}

	void finalSkills() {
		// Raise arm
		setArmStage(3);
		setIntakeState(0);

		// Climb
		runFollowLinearYield();
		runFollowLinearYield();
		driveDistanceTiles(-0.5, 50);
		driveDistanceTiles(0.3, 30);
	}
}
