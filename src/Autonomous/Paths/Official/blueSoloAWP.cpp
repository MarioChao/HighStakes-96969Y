#include "Autonomous/autonPaths.h"

namespace {
	using namespace autonpaths;
	using namespace autonpaths::pathbuild;
	using namespace autonpaths::combination;

	void loadPaths(int section);

	void doAuton1();
	void doAuton2();
}

/// @brief Run the blue solo AWP.
void autonpaths::runBlueSoloAWP() {
	/* Pre auton */

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
	doAuton1();

	loadPaths(2);
	doAuton2();
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
			pushNewLinear({{7, 3}}, false, autonvals::scoreWallStakeVelocity_pct);
		} else if (section == 2) {
			// Score 2 rings
			pushNewSpline(UniformCubicSpline::fromAutoTangent(cspline::CatmullRom, {
				{4.97, 4.1}, {3.95, 4.11}, {3.28, 4.62}, {3.22, 5.42}, {3.23, 6.23}
			}));

			// Score 2 rings
			pushNewSpline(UniformCubicSpline::fromAutoTangent(cspline::CatmullRom, {
				{2.27, 5.44}, {3.28, 5.41}, {4.11, 4.84}, {4.84, 3.46}, {5.08, 2.56}, {5.09, 1.64}
			}));

			// Grab goal
			pushNewSpline(UniformCubicSpline::fromAutoTangent(cspline::CatmullRom, {
				{6.1, 2.89}, {5.01, 2.47}, {3.99, 1.99}, {2.97, 1.57}
			}), true, 70);

			// Score 1 ring
			pushNewLinear({{4, 0.95}});

			// Touch ladder
			pushNewLinear({{3.41, 1.82}}, 50);
		}
	}

	void doAuton1() {
		// Intake filter at hood
		setIntakeFilterEnabled(0);
		setIntakeStoreRing(1);

		// Score on alliance wall stake
		runFollowLinearYield();
		setIntakeStoreRing(0, 0.5);
		wait(50, msec);
		runFollowLinearYield();
		driveDistanceTiles(-0.4);
		setIntakeState(-1);
	}

	void doAuton2() {
		// Grab goal
		grabGoalAt(3.95, 4.1);

		// Score
		setIntakeState(1);
		turnToAngle(-90);
		runFollowSpline();
		waitUntil(_pathFollowCompleted);

		// Score
		turnToAngle(90);
		runFollowSpline();
		waitUntil(_pathFollowCompleted);
		setGoalClampState(0);

		// Grab goal
		turnToAngle(65);
		runFollowSpline();
		waitUntil(_pathFollowDistanceRemaining_tiles < 0.15);
		setGoalClampState(1);
		waitUntil(_pathFollowCompleted);

		// Score
		runFollowLinearYield();

		// Touch ladder
		setArmStage(3);
		runFollowLinearYield();
	}
}
