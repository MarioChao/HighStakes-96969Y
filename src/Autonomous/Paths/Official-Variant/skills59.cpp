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
			pushNewLinear({{2, 1.15}, {3.9, 1.1}});

			// Score on wall stake
			pushNewLinear({{3.0, 1.2}}, true);
			pushNewLinear({{3, -1}}, false, autonvals::scoreNeutralWallStakeVelocity_pct);

			// Score 3 rings
			// pushNewLinear({{1, 1.1}, {0.5, 1.1}, {1.45, 0.35}});
			pushNewLinear({{0.5, 1.1}, {1.45, 0.35}});

			// Place goal at corner
			pushNewLinear({{0.39, 0.39}}, true);
		} else if (section == 2) {
			// Redirect 1 ring
			pushNewLinear({{2, 4}});

			// Score 2 rings
			pushNewLinear({{2, 4.85}, {3.9, 4.9}});

			// Score on wall stake
			pushNewLinear({{3, 4.8}}, true);
			pushNewLinear({{3, 7}}, false, autonvals::scoreNeutralWallStakeVelocity_pct);

			// Score 3 rings
			// pushNewLinear({{1, 4.9}, {0.5, 4.9}, {1.45, 5.65}});
			pushNewLinear({{0.5, 4.9}, {1.45, 5.65}});

			// Place goal at corner
			pushNewLinear({{0.39, 5.61}}, true);
		} else if (section == 3) {
			// Redirect 1 ring
			pushNewLinear({{3, 3}});

			// Store 1 ring
			pushNewLinear({{4, 4}});

			// Score on wall stake
			pushNewLinear({{7, 3}}, false, autonvals::scoreAllianceWallStakeVelocity_pct);

			// Score 2 rings
			pushNewLinear({{4.38, 4.03}});
			pushNewLinear({{5, 5.47}});

			// Score 3 rings
			pushNewLinear({{4.65, 4}, {4.8, 2.75}}, true);
			pushNewLinear({{4, 2}});
			pushNewLinear({{4.95, 1.05}});
			pushNewLinear({{4.95, 0.55}});

			// Place goal at corner
			pushNewLinear({{6, 0}}, true);

		} else if (section == 4) {
			// Prepare to grab goal
			pushNewLinear({{4.37, 1.96}});

			// Place goal at corner
			pushNewLinear({{6, 6}}, true);
		} else if (section == 5) {
			// Climb on ladder
			pushNewLinear({{3.8, 3.8}});
			pushNewLinear({{3, 3}}, false, 50);
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
		// setIntakeToArm(1);
		setIntakeState(1);
		runFollowLinearYield();

		// Score
		setIntakeToArm(0, 0.5);
		runFollowLinearYield();

		// Wall stake
		if (false) {
			setArmStage(3);
			runFollowLinearYield();
			runFollowLinearYield();
			// Odometry wall align
			mainOdometry.setPosition(mainOdometry.getX(), 0.45);
			driveDistanceTiles(-0.5);
		} else {
			autonpaths::pathbuild::linearIndex += 2;
		}

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
		grabGoalAt(1.1, 4.1);

		// Redirect
		// setIntakeToArm(1);
		setIntakeState(1);
		runFollowLinearYield();

		// Score
		setIntakeToArm(0, 0.5);
		runFollowLinearYield();

		// Wall stake
		if (false) {
			setArmStage(3);
			setIntakeState(1);
			runFollowLinearYield();
			runFollowLinearYield();
			// Odometry wall align
			mainOdometry.setPosition(mainOdometry.getX(), 5.55);
			driveDistanceTiles(-0.5);
			setArmStage(0, 1);
		} else {
			autonpaths::pathbuild::linearIndex += 2;
		}

		// Score
		runFollowLinearYield();

		// Place goal
		runFollowLinearYield();
		setGoalClampState(0);
		driveDistanceTiles(0.5);
	}

	void thirdCorner() {
		// Redirect
		// setIntakeToArm(1, 0.5);
		// setIntakeState(1);
		runFollowLinearYield();

		// Store
		setIntakeToArm(0, 0.5);
		setIntakeStoreRing(1, 0.5);
		runFollowLinearYield();

		// Goal
		grabGoalAt(5, 3);

		// Wall stake
		if (false) {
			setArmStage(2);
			runFollowLinearYield();
			// Odometry wall align
			mainOdometry.setPosition(5.5, mainOdometry.getY());
			driveDistanceTiles(-0.5);
			setArmStage(0, 1.0);
		} else {
			autonpaths::pathbuild::linearIndex += 1;
		}

		// Score 2 rings
		setIntakeState(1);
		runFollowLinearYield();
		setSwing2State(1, 0.6);
		runFollowLinearYield();
		setSwing2State(0);

		// Score 3 rings
		runFollowLinearYield();
		runFollowLinearYield();
		runFollowLinearYield();
		runFollowLinearYield();

		// Clear corner
		setSwing2State(1);
		turnToAngleVelocity(45, 60);
		setSwing2State(0);

		// Place goal
		runFollowLinearYield();
		setGoalClampState(0);
		driveDistanceTiles(0.5);
	}

	void fourthCorner() {
		// Fake grab goal
		runFollowLinearYield();
		setGoalClampState(1);
		grabGoalAt(5.46, 3.95);

		// Place goal
		runFollowLinearYield();
		// setGoalClampState(0);
		driveDistanceTiles(0.5);
	}

	void finalSkills() {
		// Raise arm
		setArmStage(3);
		setIntakeState(0);

		// Climb
		runFollowLinearYield();
		runFollowLinearYield();
		driveDistanceTiles(0.5, 50);
		driveDistanceTiles(-0.4, 30);
		driveDistanceTiles(0.3, 20);
		driveDistanceTiles(-0.2, 15);
	}
}
