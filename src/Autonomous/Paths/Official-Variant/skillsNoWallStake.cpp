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
void autonpaths::runAutonSkillsNoWallStake() {
	/* Pre skills */

	// Timer
	_autonTimer.reset();

	// Set position and rotation
	mainOdometry.setPosition(0.792, 3);
	setRobotRotation(-90.0);

	// Set config
	setDifferentialUseRelativeRotation(true);

	// Wait for arm reset
	waitUntil(isArmResetted());


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

			// Score 1 ring
			pushNewLinear({{1.93, 2}});

			// Score 2 rings
			pushNewLinear({{2.01, 1.1}, {3.93, 1.1}});

			// Score 3 rings
			pushNewLinear({{0.5, 1.1}, {1.35, 0.37}});

			// Place goal at corner
			pushNewLinear({{0.39, 0.39}}, true);
		} else if (section == 2) {
			// Score 3 rings
			pushNewLinear({{2, 4}, {2, 4.9}, {3.93, 4.9}});

			// Score 3 rings
			pushNewLinear({{0.5, 4.9}, {1.35, 5.63}});

			// Place goal at corner
			pushNewLinear({{0.39, 5.7}}, true);
		} else if (section == 3) {
			// Store 1 ring
			pushNewLinear({{3.18, 5}, {4, 4}});

			// Score 2 rings
			pushNewLinear({{4.38, 4.03}});
			pushNewLinear({{5, 5.47}});

			// Score 3 rings
			pushNewLinear({{4.65, 4}, {4.75, 2.8}}, true);
			pushNewLinear({{4, 2}, {4.95, 1.05}, {4.95, 0.55}});

			// Place goal at corner
			pushNewLinear({{5.75, 0.25}}, true);
		} else if (section == 4) {
			// Prepare to grab goal
			pushNewLinear({{4.37, 1.96}});

			// Place goal at corner
			pushNewLinear({{5.9, 5.9}}, true);
		} else if (section == 5) {
			// Climb on ladder
			pushNewLinear({{3.78, 3.78}});
			pushNewLinear({{3, 3}}, false, 50);
			// pushNewLinear({{3, 3}}, false, 65);
		}
	}

	void firstCorner() {
		// Wall stake
		setArmStage(2);
		wait(600, msec);
		runFollowLinearYield();
		driveDistanceTiles(-0.5);

		// Goal
		grabGoalAt(1, 2);

		// Score 1 ring
		setIntakeState(1);
		runFollowLinearYield();

		// Score 2 rings
		runFollowLinearYield();

		// Score 3 rings
		runFollowLinearYield();

		// Place goal
		runFollowLinearYield();
		setGoalClampState(0);
		driveDistanceTiles(0.5);
	}

	void secondCorner() {
		// Goal
		grabGoalAt(1, 4.1);

		// Score 3 rings
		setIntakeStoreRing(0);
		setIntakeState(1);
		runFollowLinearYield();

		// Score 3 rings
		runFollowLinearYield();

		// Place goal
		runFollowLinearYield();
		setGoalClampState(0);
		driveDistanceTiles(0.5);
	}

	void thirdCorner() {
		// Store
		setIntakeStoreRing(1, 0.5);
		runFollowLinearYield();

		// Goal
		grabGoalAt(5, 3);

		// Score
		setIntakeStoreRing(0);
		setIntakeState(1);

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
		setGoalClampState(0);
		driveDistanceTiles(0.5);
	}

	void fourthCorner() {
		// Grab goal
		runFollowLinearYield();
		grabGoalAt(5.46, 3.95);

		// Place goal
		runFollowLinearYield();
		setGoalClampState(0);
		driveDistanceTiles(0.5);
	}

	void finalSkills() {
		setArmStage(3);
		setIntakeState(0);
		runFollowLinearYield();
		runFollowLinearYield();
		driveDistanceTiles(-0.5, 50);
		driveDistanceTiles(0.3, 30);
	}
}
