#include "Autonomous/autonPaths.h"

#include "Utilities/generalUtility.h"
#include "Utilities/angleUtility.h"

#include "Simulation/robotSimulator.h"

namespace {
	using namespace autonpaths;
	using namespace autonpaths::pathbuild;

	void loadPaths(int section);

	void firstGoal();
	void secondGoal();
	void thirdGoal();
	void fourthGoal();
	void finalSkills();
}

/// @brief Run the skills autonomous.
void autonpaths::runAutonSkills() {
	/* Pre skills */

	// Timer
	_autonTimer.reset();

	// Set position and rotation
	mainOdometry.printDebug();
	mainOdometry.setPosition(0.792, 3);
	setRobotRotation(-90.0);
	mainOdometry.printDebug();

	if (mainUseSimulator) {
		robotSimulator.position = Vector3(0.792, 3);
		robotSimulator.angularPosition = genutil::toRadians(angle::swapFieldPolar_degrees(-90));
	}

	// Set config
	setDifferentialUseRelativeRotation(true);

	// Wait for arm reset
	waitUntil(isArmResetted());


	/* Skills */
	loadPaths(1);
	firstGoal();

	loadPaths(2);
	secondGoal();

	loadPaths(3);
	thirdGoal();

	loadPaths(4);
	fourthGoal();

	loadPaths(5);
	finalSkills();
}

namespace {
	void loadPaths(int section) {
		// Clear
		clearLinear();
		clearSplines();

		if (section == 1) {
			// Redirect 1 ring
			pushNewLinear({{2, 2.01}});

			// Score 3 rings
			// pushNewLinear({{2.01, 1.02}, {3, 0.51}, {3.99, 1}});
			pushNewLinear({{2.01, 1.02}});
			pushNewSpline(UniformCubicSpline::fromAutoTangent(cspline::CatmullRom, {
				{0.9, 2.33}, {2.01, 1.02}, {2.98, 0.53}, {4.07, 1.08}, {5.06, 2.31}
			}));

			// Prepare to score on neutral wall stake
			pushNewLinear({{3, 1.2}}, true);
			pushNewLinear({{3, 0.7}});

			// Score 3 rings
			pushNewLinear({{1.73, 1}, {1, 1}, {0.55, 1}, {0.99, 0.53}});
		} else if (section == 2) {
			// Redirect middle ring
			pushNewLinear({{2.01, 2.03}, {3.01, 3}});

			// Store ring
			pushNewLinear({{4, 2.01}});

			// Grab mobile goal
			pushNewLinear({{5, 3.01}}, true, 60.0);

			// Score 3 rings at top
			pushNewLinear({{4.6, 3.7}, {5.01, 4.99}, {4.98, 5.4}, {5.51, 4.99}});

			// Score 2 rings at bottom
			pushNewLinear({{4.72, 4.21}, {5.08, 1.48}, {4.94, 0.49}});

			// Redirect 1 ring
			pushNewLinear({{5.43, 0.96}});

			// Face alliance wall stake
			pushNewLinear({{4.61, 1.56}, {4.71, 3.01}, {5.58, 3.02}});
		} else if (section == 3) {
			// Push mobile goal to corner
			pushNewLinear({{5.4, 3.6}, {5.68, 5.5}}, true);
		} else if (section == 4) {
			// Redirect 1 ring and store 1 ring
			pushNewLinear({{4.03}});
			pushNewSpline(UniformCubicSpline::fromAutoTangent(cspline::CatmullRom, {
				{8.4, 4.98}, {4.01, 5.04}, {1.93, 3.91}, {0.21, 0.29}
			}));

			// Grab goal
			pushNewLinear({{1.02, 4.01}}, true, 60.0);

			// Go to neutral wall stake
			pushNewLinear({{3.0, 4.7}});

			// Score 1 ring
			pushNewLinear({{3.95, 4.07}});

			// Score 3 rings
			pushNewLinear({{2.98, 4.94}, {1.9, 4.94}});
			pushNewLinear({{0.49, 4.94}}, 60.0);

			// Redirect 1 ring
			pushNewLinear({{0.98, 5.45}});
		} else if (section == 5) {
			// Go to alliance wall stake
			pushNewLinear({{0.8, 3}});

			// Climb on ladder
			pushNewSpline(UniformCubicSpline::fromAutoTangent(cspline::CatmullRom, {
				{0.13, 3.81}, {0.51, 2.99}, {1.38, 1.87}, {2.24, 2.35}, {3.23, 3.42}
			}));
		}
	}

	void firstGoal() {
		/* Score alliance wall stake and grab goal */

		// Score preload on alliance wall stake
		setArmStage(2);
		task::sleep(600);
		driveAndTurnDistanceTiles(1.0, -90.0, 100.0, 100.0, defaultMoveTilesErrorRange, 0.5);
		driveAndTurnDistanceTiles(-0.5, -90.0, 70.0, 100.0, defaultMoveTilesErrorRange, 0.5);
		setArmStage(1, 0.5);

		// Go to goal
		turnToAngle(-10, 0, defaultTurnAngleErrorRange, 1.0);
		driveAndTurnDistanceTiles(-1.3, 0, 80.0, 100.0, defaultMoveTilesErrorRange, 1.0);

		// Grab goal
		setGoalClampState(1);
		wait(200, msec);


		/* Redirect 1 ring */

		// Start intake
		setIntakeState(1);
		setIntakeToArm(1);

		// Follow path
		runFollowLinearYield();

		// Remove redirect
		setIntakeToArm(0, 0.5);


		/* Score 3 rings */

		// Follow path
		runFollowLinearYield();

		// Follow path
		turnToAngle(120);
		runFollowSpline();

		// Wait
		waitUntil(_pathFollowCompleted);


		/* Score on neutral wall stake */

		// Raise arm
		setArmStage(3);

		// Follow path
		runFollowLinearYield();
		runFollowLinearYield();

		// Drive forward and score
		turnToAngle(180.0);
		setIntakeState(0);
		driveAndTurnDistanceTiles(1.5, 180.0, 100.0, 100.0, defaultMoveTilesErrorRange, 1.0);
		driveAndTurnDistanceTiles(-0.5, 180.0, 70.0, 100.0, defaultMoveTilesErrorRange, 0.5);


		/* Score 3 rings */

		// Start intake
		setIntakeState(1);

		// Follow path
		runFollowLinearYield();


		/* Place mobile goal in corner */

		// Release goal and push to corner
		turnToAngle(60);
		setGoalClampState(0);
		setIntakeState(0);
		driveAndTurnDistanceTiles(-1.0, 60, 100.0, 100.0, defaultMoveTilesErrorRange, 1.0);
		driveAndTurnDistanceTiles(0.5, 45, 80.0, 100.0, defaultMoveTilesErrorRange, 1.0);
	}

	void secondGoal() {
		/* Redirect middle ring */

		// Start intake
		setIntakeState(1);
		setIntakeToArm(1);

		// Follow path
		runFollowLinearYield();

		// Remove redirect
		setIntakeToArm(0, 1.0);
		setIntakeState(0, 1.0);


		/* Store 1 ring */

		// Store ring
		setIntakeStoreRing(1, 1.0);

		// Follow path
		runFollowLinearYield();

		// Remove store
		setIntakeStoreRing(0, 1.0);


		/* Grab mobile goal */

		// Follow path
		runFollowLinearYield();

		// Grab goal
		setGoalClampState(1);
		setArmStage(2);
		wait(200, msec);


		/* Score on alliance wall stake */

		// Turn
		turnToAngle(90);

		// Score on alliance wall stake
		driveAndTurnDistanceTiles(1.5, 90.0, 100.0, 100.0, defaultMoveTilesErrorRange, 1.0);
		driveAndTurnDistanceTiles(-0.5, 90.0, 80.0, 100.0, defaultMoveTilesErrorRange, 0.5);
		setArmStage(0);


		/* Score 3 rings */

		// Start intake & score
		setIntakeState(1);

		// Follow path
		runFollowLinearYield();


		/* Score 2 rings and redirect 1 ring */

		// Turn
		turnToAngle(-135);

		// Follow path
		runFollowLinearYield();

		// Redirect ring
		setIntakeToArm(1, 0.5);

		// Follow path
		runFollowLinearYield();


		/* Place mobile goal in corner */

		// Release goal and push to corner
		turnToAngle(-25);
		setGoalClampState(0);
		driveAndTurnDistanceTiles(-1.0, -25, 100.0, 100.0, defaultMoveTilesErrorRange, 1.0);
		driveAndTurnDistanceTiles(0.5, -45, 80.0, 100.0, defaultMoveTilesErrorRange, 1.0);


		/* Score on alliance wall stake */

		// Raise arm
		setIntakeState(0);
		setArmStage(2);

		// Follow path
		runFollowLinearYield();

		// Score on alliance wall stake
		turnToAngle(90.0);
		driveAndTurnDistanceTiles(1.5, 90.0, 100.0, 100.0, defaultMoveTilesErrorRange, 0.8);
		driveAndTurnDistanceTiles(-0.5, 90.0, 80.0, 100.0, defaultMoveTilesErrorRange, 0.5);
		setArmStage(3);
	}

	void thirdGoal() {
		/* Push mobile goal to corner */

		// Follow path
		runFollowLinearYield();

		// Move forward a bit
		driveAndTurnDistanceTiles(0.5, -170, 80.0, 100.0, defaultMoveTilesErrorRange, 0.5);
		setArmHangState(0);
	}

	void fourthGoal() {
		/* Redirect 1 ring and store 1 ring */

		// Start intake and redirect
		turnToAngle(-105);
		setIntakeState(1);
		setIntakeToArm(1);

		// Follow path
		runFollowLinearYield();

		// Remove redirect and start to store
		setIntakeToArm(0, 0.5);
		setIntakeStoreRing(1, 0.5);

		// Follow path
		runFollowSpline();

		// Wait
		waitUntil(_pathFollowCompleted);


		/* Grab mobile goal */

		// Follow path
		runFollowLinearYield();

		// Grab goal
		setGoalClampState(1);
		wait(200, msec);


		/* Score on neutral wall stake */

		// Start scoring and raise arm
		setIntakeState(1);
		setArmStage(3);

		// Follow path
		runFollowLinearYield();

		// Score on neutral wall stake
		turnToAngle(0);
		driveAndTurnDistanceTiles(1.5, 0.0, 100.0, 100.0, defaultMoveTilesErrorRange, 1.0);
		driveAndTurnDistanceTiles(-0.5, 0.0, 80.0, 100.0, defaultMoveTilesErrorRange, 0.5);
		setArmStage(0, 1.0);


		/* Score 1 ring */

		// Follow path
		runFollowLinearYield();


		/* Score 3 rings and redirect 1 ring */

		// Follow path
		runFollowLinearYield();
		runFollowLinearYield();

		// Redirect ring
		setIntakeToArm(1, 0.5);
		driveAndTurnDistanceTiles(1.0, 55, 80.0, 100.0, defaultMoveTilesErrorRange, 1.0);


		/* Place mobile goal in corner */

		// Release goal and push to corner
		turnToAngle(105);
		setGoalClampState(0);
		setIntakeState(0, 1.0);
		driveAndTurnDistanceTiles(-1.0, 105, 100.0, 100.0, defaultMoveTilesErrorRange, 1.0);
		driveAndTurnDistanceTiles(0.5, 135, 80.0, 100.0, defaultMoveTilesErrorRange, 1.0);
	}

	void finalSkills() {
		/* Score on alliance wall stake */

		// Raise arm
		setArmStage(2);

		// Follow path
		runFollowLinearYield();

		// Score on alliance wall stake
		turnToAngle(-90);
		driveAndTurnDistanceTiles(1.0, -90.0, 100.0, 100.0, defaultMoveTilesErrorRange, 1.0);
		driveAndTurnDistanceTiles(-0.5, -90.0, 80.0, 100.0, defaultMoveTilesErrorRange, 0.5);


		/* Climb on ladder */

		// Turn and raise arm
		turnToAngle(145);
		setArmStage(3);

		// Follow path
		runFollowSpline();

		// Wait
		waitUntil(_pathFollowCompleted);

		// Climb
		turnToAngle(45);
		driveAndTurnDistanceTiles(1.0, 45, 40.0, 100.0);
	}
}
