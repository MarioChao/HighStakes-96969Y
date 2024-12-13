#include "Autonomous/autonPaths.h"

namespace {
	using namespace autonpaths;
	using namespace autonpaths::pathbuild;

	void loadPaths(int section);

	void firstCorner();
	void secondCorner();
	void thirdCorner();
	void fourthCorner();
	void finalSkills();
}

/// @brief Run the autonomous skills.
void autonpaths::runAutonSkills() {
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
			pushNewLinear({{0, 3}});

			// Grab goal
			pushNewLinear({{1, 2}}, true, 70);

			// Redirect 1 ring
			pushNewLinear({{1.93, 2}});

			// Score 2 rings
			pushNewLinear({{2.01, 1.06}, {3.93, 1}});

			// Go to wall stake
			pushNewLinear({{3.0, 1.2}});

			// Score on wall stake
			pushNewLinear({{3, 0}});

			// Score 3 rings
			pushNewLinear({{1.71, 1.05}, {0.5, 1.05}, {1.19, 0.37}});

			// Place goal at corner
			pushNewLinear({{0.39, 0.39}}, true);
		} else if (section == 2) {
			// Redirect 1 ring
			pushNewLinear({{3, 3}});

			// Store 1 ring
			pushNewLinear({{2, 4}});

			// Grab goal
			pushNewLinear({{1.1, 4.01}}, true, 70);

			// Score 2 ring & score on wall stake
			pushNewLinear({{2, 4.9}, {3, 4.9}, {3, 6}});

			// Score 3 rings
			pushNewLinear({{1.04, 5.52}, {1.03, 5.01}, {0.49, 5}});

			// Place goal at corner
			pushNewLinear({{0.39, 5.61}}, true);
		} else if (section == 3) {
			// Redirect 1 ring
			pushNewLinear({{4, 5}});

			// Store 1 ring
			pushNewLinear({{4, 4}});

			// Grab goal
			pushNewLinear({{5, 3}}, true, 70);

			// Score on wall stake
			pushNewLinear({{6, 3}});

			// Place goal at corner
			pushNewLinear({{5.5, 3.9}, {5.61, 5.61}}, true);

		} else if (section == 4) {
			// Store 1 ring
			pushNewLinear({{5, 5}});

			// Grab goal
			pushNewLinear({{5, 3}}, true, 70);

			// Score 3 rings
			pushNewLinear({{4, 2}, {4.95, 1.05}, {4.95, 0.55}});

			// Reposition
			pushNewLinear({{4.95, 1}}, true);

			// Score 1 ring
			pushNewLinear({{5.55, 1.04}});

			// Place goal at corner
			pushNewLinear({{5.61, 0.39}}, true);
		} else if (section == 5) {
			// Climb on ladder
			pushNewLinear({{3.78, 2.22}});
			pushNewLinear({{3, 3}}, false, 50);
		}
	}

	void firstCorner() {
		// Wall stake
		setArmStage(2);
		wait(600, msec);
		runFollowLinearYield();
		driveDistanceTiles(-0.5);

		// Goal
		runFollowLinearYield();
		setGoalClampState(1);

		// Redirect
		setIntakeToArm(1);
		setIntakeState(1);
		runFollowLinearYield();

		// Score
		setIntakeToArm(0, 0.5);
		runFollowLinearYield();

		// Wall stake
		runFollowLinearYield();
		runFollowLinearYield();

		// Score
		runFollowLinearYield();

		// Place goal
		runFollowLinearYield();
		setGoalClampState(0);
		driveDistanceTiles(0.5);
	}

	void secondCorner() {
		// Redirect
		setIntakeToArm(1);
		setIntakeState(1);
		runFollowLinearYield();

		// Store
		setIntakeStoreRing(1, 0.5);
		runFollowLinearYield();

		// Goal
		runFollowLinearYield();
		setGoalClampState(1);

		// Score + wall stake
		setArmStage(3);
		setIntakeState(1);
		runFollowLinearYield();
		driveDistanceTiles(-0.5);
		setArmStage(0, 1.0);

		// Score
		runFollowLinearYield();

		// Place goal
		runFollowLinearYield();
		setGoalClampState(0);
		driveDistanceTiles(0.5);
	}

	void thirdCorner() {
		// Redirect
		setIntakeToArm(1);
		setIntakeState(1);
		runFollowLinearYield();

		// Store
		setIntakeStoreRing(1, 0.5);
		runFollowLinearYield();

		// Goal
		runFollowLinearYield();
		setGoalClampState(1);

		// Release goal
		setIntakeState(1);
		setArmStage(2);
		turnToAngle(90);
		wait(200, msec);
		setGoalClampState(0);
		setIntakeState(0);

		// Wall stake
		runFollowLinearYield();
		driveDistanceTiles(-0.5);
		setArmStage(0, 1.0);

		// Place goal
		runFollowLinearYield();
		driveDistanceTiles(0.5);
	}

	void fourthCorner() {
		// Store
		setIntakeStoreRing(1);
		runFollowLinearYield();

		// Goal
		runFollowLinearYield();
		setGoalClampState(1);

		// Score
		setIntakeStoreRing(0);
		setIntakeState(1);
		runFollowLinearYield();

		// Reposition
		runFollowLinearYield();

		// Score
		runFollowLinearYield();

		// Place goal
		runFollowLinearYield();
		setGoalClampState(0);
		driveDistanceTiles(0.5);
	}

	void finalSkills() {
		setArmStage(3);
		runFollowLinearYield();
		runFollowLinearYield();
	}
}
