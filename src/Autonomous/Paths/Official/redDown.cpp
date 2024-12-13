#include "Autonomous/autonPaths.h"

namespace {
	using namespace autonpaths;
	using namespace autonpaths::pathbuild;

	void loadPaths(int section);

	void doAuton();
}

/// @brief Run the 15-seconds new red-down autonomous.
void autonpaths::runAutonRedDown() {
	/* Pre skills */

	// Timer
	_autonTimer.reset();

	// Set position and rotation
	mainOdometry.printDebug();
	mainOdometry.setPosition(0.68, 1.69);
	setRobotRotation(105.0);
	mainOdometry.printDebug();

	// Set config
	setDifferentialUseRelativeRotation(true);

	// Wait for arm reset
	waitUntil(isArmResetted());


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
			// Rush goal
			pushNewLinear({{2.62, 1.22}});

			// Grab goal
			pushNewLinear({{2.05, 1.97}}, true);

			// Store 1 ring
			pushNewLinear({{2, 1}});

			// Grab goal
			pushNewLinear({{2.62, 1.26}}, true);

			// Go to corner
			pushNewLinear({{1.04, 0.41}});

			// Redirect corner ring
			pushNewLinear({{0.25, 0.25}});

			// Score alliance wall stake
			pushNewLinear({{0.64, 3}, {0.3, 3}});

			// Touch ladder
			pushNewSpline(UniformCubicSpline::fromAutoTangent(cspline::CatmullRom, {
				{0.21, 4.84}, {0.53, 2.98}, {2.45, 2.41}, {4.48, 3.95}
			}), true);
		}
	}

	void doAuton() {
		/* Rush middle goal */

		// Follow path
		runFollowLinearYield();

		// Deploy
		setSwing2State(1);

		// Go back
		driveDistanceTiles(-0.5);
		setSwing2State(0);


		/* Grab goal*/

		// Follow path
		runFollowLinearYield();

		// Grab goal
		setGoalClampState(1);
		wait(200, msec);


		/* Score 1 ring */

		// Score preload
		setIntakeState(1);

		// Release goal
		wait(1, sec);
		setGoalClampState(0);


		/* Store 1 ring */

		// Start storing
		setIntakeStoreRing(1, 0.3);

		// Follow path
		runFollowLinearYield();


		/* Grab goal */

		// Follow path
		runFollowLinearYield();

		// Grab goal
		setGoalClampState(1);
		wait(200, msec);

		// Start scoring
		setIntakeState(1);


		/* Sweep corner */

		// Follow path
		runFollowLinearYield();

		// Deploy swing
		setSwingState(1);
		// setIntakeState(0);

		// Swing out rings
		driveAndTurnDistanceTiles(0.7, -135, 60, 100, 0.5);
		turnToAngleVelocity(-80, 50);
		setSwingState(0);


		/* Redirect corner */

		// Start redirect
		setIntakeState(1);
		setIntakeToArm(1);

		// Follow path
		runFollowLinearYield();

		// Back up
		driveAndTurnDistanceTiles(-0.5, -135, 100.0, 100.0, 1.0);


		/* Score alliance wall stake */

		// Raise arm
		setArmStage(2, 0.5);

		// Follow path
		runFollowLinearYield();


		/* Touch ladder */

		// Lower arm
		setArmStage(0, 1.0);

		// Follow path
		runFollowSpline();

		// Wait
		waitUntil(_pathFollowCompleted);

		// Stop intake
		waitUntil(_autonTimer.value() > 14.5);
		setIntakeState(0);
	}
}
