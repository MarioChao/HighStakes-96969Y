#include "Autonomous/autonPaths.h"

namespace {
	using namespace autonpaths;
	using namespace autonpaths::pathbuild;

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
	mainOdometry.setPosition(6 - 0.8, 3.5);
	setRobotRotation(120.0);
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
			// Grab goal
			pushNewLinear({{6 - (2), 4}}, true, 60.0);

			// Score 2 rings
			pushNewSpline(UniformCubicSpline::fromAutoTangent(cspline::CatmullRom, {
				{6 - (0.76), 4.01}, {6 - (2.06), 4.02}, {6 - (2.75), 4.49}, {6 - (2.76), 5.5}, {6 - (1.84), 6.28}
			}), false, 0.7);

			// Go to corner
			pushNewLinear({{6 - (2.01), 5.02}, {6 - (0.45), 5.4}});

			// Score corner
			pushNewLinear({{6 - (0.19), 5.83}});

			// Score 1 ring
			pushNewLinear({{6 - (1.02), 3.38}, {6 - (1.02), 2.51}});

			// Touch ladder
			pushNewSpline(UniformCubicSpline::fromAutoTangent(cspline::CatmullRom, {
				{6 - (0.99), 1.34}, {6 - (1.02), 2.49}, {6 - (1.4), 3.6}, {6 - (2.61), 3.26}, {6 - (3.97), 2.03}
			}), true);
		}
	}

	void doAuton() {
		/* Score alliance wall stake and grab goal */

		// Score preload on alliance wall stake
		setArmStage(2);
		task::sleep(600);
		driveAndTurnDistanceTiles(0.45, -(-120.0), 40.0, 100.0, 0.5);
		driveAndTurnDistanceTiles(-0.5, -(-120.0), 40.0, 100.0, 1.5);
		setArmStage(0);

		// Follow path
		runFollowLinearYield();

		// Grab goal
		setGoalClampState(1);
		wait(200, msec);


		/* Score 2 rings */

		// Start intake
		turnToAngle(-(80));
		setIntakeState(1);

		// Follow path
		runFollowSpline();

		// Wait
		waitUntil(_pathFollowCompleted);


		/* Score 1 ring */

		// Follow path
		runFollowLinearYield();


		/* Sweep corner */

		// Deploy swing
		turnToAngle(-(-60));
		setSwingState(1);

		// Swing out rings
		driveAndTurnDistanceTiles(0.7, -(-60), 60, 100, 0.5);
		turnToAngleVelocity(-(0), 50);
		setSwingState(0);

		/* Score corner */

		// Follow path
		runFollowLinearYield();

		// Back up
		driveAndTurnDistanceTiles(-0.5, -(-45), 100.0, 100.0, 1.0);


		/* Score 1 ring */

		// Follow path
		runFollowLinearYield();


		/* Touch ladder */

		// Follow path
		runFollowSpline();

		// Wait
		waitUntil(_pathFollowCompleted);

		// Stop intake
		waitUntil(_autonTimer.value() > 14.5);
		setIntakeState(0);
	}
}
