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

	bool skipWallStakes = true;
	bool skipLastWallStake = true;
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
			pushNewLinear({{0, 3}}, false, autonvals::scoreAllianceWallStakeVelocity_pct);

			// Redirect 1 ring
			pushNewLinear({{1.93, 2}});

			// Score 2 rings
			pushNewLinear({{2.01, 1.06}, {3.93, 1}});

			// Go to wall stake
			pushNewLinear({{3.0, 1.2}}, true);

			// Score on wall stake
			pushNewLinear({{3, 0}}, false, autonvals::scoreAllianceWallStakeVelocity_pct);

			// Score 3 rings
			pushNewLinear({{1.71, 1.05}, {0.5, 1.05}, {1.3, 0.37}});

			// Place goal at corner
			pushNewLinear({{0.39, 0.39}}, true);
		} else if (section == 2) {
			// Redirect 1 ring
			pushNewLinear({{3, 3}});

			// Store 1 ring
			pushNewLinear({{2, 4}});

			// Score 2 ring & score on wall stake
			pushNewLinear({{2, 4.9}, {3, 4.8}});
			pushNewLinear({{3, 6}}, false, autonvals::scoreAllianceWallStakeVelocity_pct);

			// Score 3 rings
			// pushNewLinear({{1.04, 5.52}, {1.03, 5.01}, {0.49, 5}});
			pushNewLinear({{1.71, 5}, {0.5, 5}, {1.3, 5.63}});

			// Place goal at corner
			pushNewLinear({{0.39, 5.61}}, true);
		} else if (section == 3) {
			// Redirect 1 ring
			pushNewLinear({{4, 5}});

			// Store 1 ring
			pushNewLinear({{4, 4}});

			// Score on wall stake
			pushNewLinear({{6.3, 3}}, false, autonvals::scoreAllianceWallStakeVelocity_pct);

			// Place goal at corner
			// pushNewLinear({{5.5, 3.9}, {5.7, 5.7}}, true);
			pushNewLinear({{6, 5.7}}, true);

		} else if (section == 4) {
			// Store 1 ring
			pushNewLinear({{5, 5}});

			// Score 3 rings
			pushNewLinear({{4, 2}, {4.95, 1.05}, {4.95, 0.55}});

			// Reposition
			// pushNewLinear({{4.8, 1}}, true);

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
		grabGoalAt(1, 2);

		// Redirect
		if (!skipWallStakes) setIntakeToArm(1);
		setIntakeState(1);
		runFollowLinearYield();

		// Score
		setIntakeToArm(0, 0.5);
		runFollowLinearYield();

		// Wall stake
		if (!skipWallStakes) {
			setArmStage(3);
			runFollowLinearYield();
			runFollowLinearYield();
			mainOdometry.setPosition(3.0, 0.33);
			driveDistanceTiles(-0.5);
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

	void secondCorner() {
		// Redirect
		if (!skipWallStakes) setIntakeToArm(1);
		else setArmStage(0);
		setIntakeState(1);
		runFollowLinearYield();

		// Store
		setIntakeToArm(0, 0.5);
		setIntakeStoreRing(1, 0.5);
		runFollowLinearYield();

		// Goal
		grabGoalAt(0.9, 4);

		// Score + wall stake
		if (!skipWallStakes) {
			setArmStage(3);
			setIntakeState(1);
			runFollowLinearYield();
			runFollowLinearYield();
			mainOdometry.setPosition(3.0, 5.67);
			driveDistanceTiles(-0.5);
			setArmStage(0, 1.0);
		} else {
			setIntakeState(1);
			driveTurnToFace_tiles(2, 5);
			turnToFace_tiles(3, 5.4);
			driveTurnToFace_tiles(3, 5.4);
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
		if (!skipLastWallStake) setIntakeToArm(1, 0.5);
		setIntakeState(1);
		runFollowLinearYield();

		// Store
		setIntakeToArm(0, 0.5);
		setIntakeStoreRing(1, 0.5);
		runFollowLinearYield();

		// Goal
		grabGoalAt(4.9, 2.8);

		// Release goal
		setIntakeState(1);
		setArmStage(2);
		turnToAngle(90);
		wait(200, msec);
		setGoalClampState(0);
		setIntakeState(0);

		// Wall stake
		if (!skipLastWallStake) {
			runFollowLinearYield();
			driveDistanceTiles(-0.5);
			setArmStage(0, 1.0);
		} else autonpaths::pathbuild::linearIndex += 1;

		// Place goal
		runFollowLinearYield();
		driveDistanceTiles(0.2);
	}

	void fourthCorner() {
		// Store
		setIntakeStoreRing(1);
		runFollowLinearYield();

		// Goal
		grabGoalAt(4.75, 2.8);

		// Score
		setIntakeStoreRing(0);
		setIntakeState(1);
		runFollowLinearYield();

		// Reposition
		// runFollowLinearYield();

		// Score
		runFollowLinearYield();

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
	}
}
