#include "Autonomous/autonPaths.h"

#include "Utilities/generalUtility.h"
#include "Utilities/angleUtility.h"

#include "Simulation/robotSimulator.h"

namespace {
	using namespace autonpaths;

	double maxVel = 3.7 * 0.7;
	double maxAccel = 6;
	double maxDecel = 6;

	std::vector<UniformCubicSpline> splines;
	std::vector<CurveSampler> splineSamplers;
	std::vector<TrajectoryPlanner> splineTrajectoryPlans;
	std::vector<bool> willReverse;

	void pushNewSpline(UniformCubicSpline spline, bool reverse = false, double maxVel = ::maxVel);
	void loadSkillsSplines(int section);
	void runFollowSpline();

	int pathIndex;

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

	// Pre-process
	loadSkillsSplines(1);

	// Set position and rotation
	mainOdometry.setPosition(0.9, 3);
	mainOdometry.setLookAngle(-90.0);
	mainOdometry.printDebug();
	setRotation(-90.0);
	mainOdometry.restart();
	mainOdometry.printDebug();

	if (mainUseSimulator) {
		robotSimulator.position = Vector3(0.9, 3);
		robotSimulator.angularPosition = genutil::toRadians(angle::swapFieldPolar_degrees(-90));
	}

	// Set config
	setDifferentialUseRelativeRotation(true);

	// Wait for arm reset
	waitUntil(isArmResetted());

	/* Skills */
	firstGoal();

	loadSkillsSplines(2);
	secondGoal();

	loadSkillsSplines(3);
	thirdGoal();

	loadSkillsSplines(4);
	fourthGoal();

	loadSkillsSplines(5);
	finalSkills();
}

namespace {
	void pushNewSpline(UniformCubicSpline spline, bool reverse, double maxVel) {
		CurveSampler splineSampler = CurveSampler(spline)
			.calculateByResolution(spline.getTRange().second * 10);
		TrajectoryPlanner splineTrajectoryPlan = TrajectoryPlanner(splineSampler.getDistanceRange().second)
			.autoSetMotionConstraints(splineSampler, 0.3, maxVel, maxAccel, maxDecel)
			.calculateMotion();
		splines.push_back(spline);
		splineSamplers.push_back(splineSampler);
		splineTrajectoryPlans.push_back(splineTrajectoryPlan);
		willReverse.push_back(reverse);
	}

	void loadSkillsSplines(int section) {
		// Clear
		splines.clear();
		splineSamplers.clear();
		splineTrajectoryPlans.clear();
		willReverse.clear();
		pathIndex = 0;

		UniformCubicSpline spline;

		if (section == 1) {
			// Alliance wall stake to mobile goal
			spline = UniformCubicSpline::fromAutoTangent(cspline::CatmullRom, {
				{-0.64, 2.86}, {0.29, 2.92}, {1, 2.04}, {0.96, 0.02}
			});
			pushNewSpline(spline, true);

			// Redirect 1 ring to arm & score 3 rings
			spline = UniformCubicSpline::fromAutoTangent(cspline::CatmullRom, {
				{-0.25, 1.99}, {1, 2.03}, {1.93, 2.25}, {2.05, 0.98}, {3, 0.57}, {4.01, 1}, {5.02, 1.75}
			});
			pushNewSpline(spline);

			// Prepare to score on neutral wall stake
			spline = UniformCubicSpline::fromAutoTangent(cspline::CatmullRom, {
				{5.08, 1.52}, {4.05, 1.02}, {3.18, 0.7}, {3.02, 1.1}, {2.98, 2.39}
			});
			pushNewSpline(spline, true);

			// Score 2 rings
			spline = UniformCubicSpline::fromAutoTangent(cspline::CatmullRom, {
				{4.54, 0.13}, {2.98, 0.45}, {1.79, 0.98}, {0.57, 1}, {-0.35, 1.02}
			});
			pushNewSpline(spline);
		} else if (section == 2) {
			// Redirect middle ring
			spline = UniformCubicSpline::fromAutoTangent(cspline::CatmullRom, {
				{-0.05, -0.09}, {0.55, 0.49}, {1.59, 1.67}, {3, 2.98}, {4.21, 4.09}
			});
			pushNewSpline(spline);

			// Store ring
			spline = UniformCubicSpline::fromAutoTangent(cspline::CatmullRom, {
				{2.03, 3.99}, {3.02, 3}, {4.01, 1.99}, {4.86, 1.18}
			});
			pushNewSpline(spline);

			// Grab mobile goal
			spline = UniformCubicSpline::fromAutoTangent(cspline::CatmullRom, {
				{1.07, 1.5}, {4.01, 2.03}, {5.02, 2.95}, {5, 4.14}
			});
			pushNewSpline(spline, true);

			// Face alliance wall stake
			spline = UniformCubicSpline::fromAutoTangent(cspline::CatmullRom, {
				{5.85, 3}, {5.02, 3}, {4.52, 3}, {3.19, 3}
			});
			pushNewSpline(spline, true);

			// Score 2 rings at top
			spline = UniformCubicSpline::fromAutoTangent(cspline::CatmullRom, {
				{6.11, 1.9}, {5.26, 2.88}, {4.76, 4.03}, {5.01, 5}, {5, 5.47}, {5, 6.25}
			});
			pushNewSpline(spline);

			// Score 2 rings at bottom
			spline = UniformCubicSpline::fromAutoTangent(cspline::CatmullRom, {
				{6.25, 5.71}, {5.49, 5}, {4.65, 4.08}, {4.75, 2.09}, {4.99, 1.01}, {5, 0.54}, {5.02, -0.41}
			});
			pushNewSpline(spline);

			// Face alliance wall stake
			spline = UniformCubicSpline::fromAutoTangent(cspline::CatmullRom, {
				{6.16, -0.14}, {5.5, 0.59}, {4.87, 1.4}, {4.55, 2.7}, {5.04, 3.01}, {6.3, 2.91}
			});
			pushNewSpline(spline);
		} else if (section == 3) {
			// Push mobile goal to corner
			spline = UniformCubicSpline::fromAutoTangent(cspline::CatmullRom, {
				{5.3, 2.05}, {5.43, 3}, {5.66, 5.63}, {5.76, 6.36}
			});
			pushNewSpline(spline, true);
		} else if (section == 4) {
			// Redirect 1 ring and store 1 ring
			spline = UniformCubicSpline::fromAutoTangent(cspline::CatmullRom, {
				{6.11, 5.85}, {5.48, 5.43}, {3.98, 5}, {2.01, 3.99}, {0.33, 1.36}
			});
			pushNewSpline(spline);

			// Grab goal
			spline = UniformCubicSpline::fromAutoTangent(cspline::CatmullRom, {
				{3.26, 4.01}, {2.03, 4.01}, {1, 3.99}, {-0.5, 4.05}
			});
			pushNewSpline(spline, true);

			// Go to neutral wall stake
			spline = UniformCubicSpline::fromAutoTangent(cspline::CatmullRom, {
				{-0.3, 4.07}, {1.05, 4}, {2.01, 4.14}, {2.83, 4.8}, {3.02, 5.51}, {3.04, 6.35}
			});
			pushNewSpline(spline);

			// Score 1 ring
			spline = UniformCubicSpline::fromAutoTangent(cspline::CatmullRom, {
				{0.85, 5.41}, {3, 5.45}, {4.02, 4.03}, {4.08, 1.01}
			});
			pushNewSpline(spline);

			// Score 3 rings
			spline = UniformCubicSpline::fromAutoTangent(cspline::CatmullRom, {
				{5.63, 2.25}, {3.96, 4.05}, {2.4, 4.93}, {0.49, 5}, {-0.42, 5}
			});
			pushNewSpline(spline);
		} else if (section == 5) {
			// Go to alliance wall stake
			spline = UniformCubicSpline::fromAutoTangent(cspline::CatmullRom, {
				{-0.19, 6.07}, {0.47, 5.53}, {1.3, 3.5}, {0.59, 3.01}, {-0.42, 3}
			});
			pushNewSpline(spline);

			// Climb on ladder
			spline = UniformCubicSpline::fromAutoTangent(cspline::CatmullRom, {
				{0.13, 3.81}, {0.51, 2.99}, {1.38, 1.87}, {2.24, 2.35}, {3.23, 3.42}
			});
			pushNewSpline(spline);
		}
	}

	void runFollowSpline() {
		autonfunctions::setSplinePath(splines[pathIndex], splineTrajectoryPlans[pathIndex], splineSamplers[pathIndex]);
		autonfunctions::followSplinePath(willReverse[pathIndex]);

		pathIndex++;
	}

	void firstGoal() {
		/* Score alliance wall stake and grab goal */

		// Score preload at alliance wall stake
		setArmStage(2);
		task::sleep(600);
		driveAndTurnDistanceTiles(0.5, -90.0, 100.0, 100.0, defaultMoveTilesErrorRange, 0.5);
		driveAndTurnDistanceTiles(-0.5, -90.0, 70.0, 100.0, defaultMoveTilesErrorRange, 0.5);
		setArmStage(1, 0.5);

		// Follow path
		runFollowSpline();

		// Wait
		waitUntil(_pathFollowCompleted);

		// Grab goal
		setGoalClampState(1);
		wait(200, msec);


		/* Redirect 1 ring and score more rings */

		// Start intake
		turnToAngle(80.0);
		setIntakeState(1);
		setIntakeToArm(1);

		// Follow path
		runFollowSpline();

		// Wait and remove redirect
		waitUntil(_trajectoryPlan.getMotionAtTime(_splinePathTimer.value())[0] >= 1.7);
		printf("No more redirect\n");
		setIntakeToArm(0);

		// Wait
		waitUntil(_pathFollowCompleted);


		/* Score on neutral wall stake */

		// Raise arm
		setArmStage(3);
		setIntakeState(0);

		// Follow path
		runFollowSpline();

		// Wait
		waitUntil(_pathFollowCompleted);

		// Drive forward and score
		turnToAngle(180.0);
		setIntakeState(0);
		driveAndTurnDistanceTiles(1.5, 180.0, 100.0, 100.0, defaultMoveTilesErrorRange, 1.0);
		driveAndTurnDistanceTiles(-0.5, 180.0, 70.0, 100.0, defaultMoveTilesErrorRange, 0.5);


		/* Score more rings */

		// Start intake
		turnToAngle(-80.0);
		setIntakeState(1);

		// Follow path
		runFollowSpline();

		// Wait
		waitUntil(_pathFollowCompleted);

		// Score 1 ring
		turnToAngle(145);
		driveAndTurnDistanceTiles(1.0, 145, 70.0, 100.0, defaultMoveTilesErrorRange, 1.0);


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
		runFollowSpline();

		// Wait
		waitUntil(_pathFollowCompleted);

		// Remove redirect
		setIntakeToArm(0);
		setIntakeState(0);


		/* Store 1 ring */

		// Store ring
		setIntakeStoreRing(1);
		turnToAngle(135);

		// Follow path
		runFollowSpline();

		// Wait
		waitUntil(_pathFollowCompleted);

		// Remove store
		setIntakeStoreRing(0);


		/* Grab mobile goal */

		// Turn
		turnToAngle(-90);

		// Follow path
		runFollowSpline();

		// Wait
		waitUntil(_pathFollowCompleted);

		// Grab goal
		setGoalClampState(1);
		wait(200, msec);


		/* Score on alliance wall stake */

		// Turn
		turnToAngle(90);
		setArmStage(2);

		// Follow path
		runFollowSpline();

		// Wait
		waitUntil(_pathFollowCompleted);

		// Score on alliance wall stake
		turnToAngle(90.0);
		driveAndTurnDistanceTiles(1.5, 90.0, 100.0, 100.0, defaultMoveTilesErrorRange, 0.8);
		driveAndTurnDistanceTiles(-0.5, 90.0, 80.0, 100.0, defaultMoveTilesErrorRange, 0.5);
		setArmStage(1, 0.5);


		/* Score 3 rings */

		// Start intake
		turnToAngle(-30);
		setIntakeState(1);

		// Follow path
		runFollowSpline();

		// Wait
		waitUntil(_pathFollowCompleted);

		// Score 1 ring
		turnToAngle(145);
		driveAndTurnDistanceTiles(1.0, 145, 100.0, 100.0, defaultMoveTilesErrorRange, 1.0);


		/* Score 2 rings and redirect 1 ring */

		// Turn
		turnToAngle(-135);

		// Follow path
		runFollowSpline();

		// Wait
		waitUntil(_pathFollowCompleted);

		// Redirect ring
		turnToAngle(40);
		setIntakeToArm(1);
		driveAndTurnDistanceTiles(1.0, 35, 80.0, 100.0, defaultMoveTilesErrorRange, 1.0);


		/* Place mobile goal in corner */

		// Release goal and push to corner
		turnToAngle(-25);
		setGoalClampState(0);
		setIntakeState(0);
		driveAndTurnDistanceTiles(-1.0, -25, 100.0, 100.0, defaultMoveTilesErrorRange, 1.0);
		driveAndTurnDistanceTiles(0.5, -45, 80.0, 100.0, defaultMoveTilesErrorRange, 1.0);


		/* Score on alliance wall stake */

		// Raise arm
		setArmStage(2);

		// Follow path
		runFollowSpline();

		// Wait
		waitUntil(_pathFollowCompleted);

		// Score on alliance wall stake
		turnToAngle(90.0);
		driveAndTurnDistanceTiles(1.5, 90.0, 100.0, 100.0, defaultMoveTilesErrorRange, 0.8);
		driveAndTurnDistanceTiles(-0.5, 90.0, 80.0, 100.0, defaultMoveTilesErrorRange, 0.5);
		setArmStage(1, 0.5);
	}

	void thirdGoal() {
		/* Push mobile goal to corner */

		// Turn
		turnToAngle(180);

		// Follow path
		runFollowSpline();

		// Wait
		waitUntil(_pathFollowCompleted);

		// Move forward a bit
		driveAndTurnDistanceTiles(0.5, -170, 80.0, 100.0, defaultMoveTilesErrorRange, 0.5);
	}

	void fourthGoal() {
		/* Redirect 1 ring and store 1 ring */

		// Start intake and redirect
		turnToAngle(-105);
		setIntakeState(1);
		setIntakeToArm(1);

		// Follow path
		runFollowSpline();

		// Wait
		waitUntil(_trajectoryPlan.getMotionAtTime(_splinePathTimer.value())[0] >= 2.0);

		// Remove redirect and start storing
		setIntakeToArm(0);
		setIntakeStoreRing(1);

		// Wait
		waitUntil(_pathFollowCompleted);


		/* Grab mobile goal */

		// Turn
		turnToAngle(90);

		// Follow path
		runFollowSpline();

		// Wait
		waitUntil(_pathFollowCompleted);

		// Grab goal
		setGoalClampState(1);
		wait(200, msec);


		/* Score on neutral wall stake */

		// Start scoring and raise arm
		setIntakeState(1);
		setArmStage(3);

		// Follow path
		runFollowSpline();

		// Wait
		waitUntil(_pathFollowCompleted);

		// Score on neutral wall stake
		turnToAngle(0);
		driveAndTurnDistanceTiles(1.0, 0.0, 100.0, 100.0, defaultMoveTilesErrorRange, 0.5);
		driveAndTurnDistanceTiles(-0.5, 0.0, 80.0, 100.0, defaultMoveTilesErrorRange, 0.5);
		setArmStage(1, 0.5);


		/* Score 1 ring */

		// Turn
		turnToAngle(90);

		// Follow path
		runFollowSpline();

		// Wait
		waitUntil(_pathFollowCompleted);


		/* Score 3 rings and redirect 1 ring */

		// Turn
		turnToAngle(-60);

		// Follow path
		runFollowSpline();

		// Wait
		waitUntil(_pathFollowCompleted);

		// Redirect ring
		turnToAngle(55);
		setIntakeToArm(1);
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

		// Turn and raise arm
		turnToAngle(160);
		setArmStage(2);

		// Follow path
		runFollowSpline();

		// Wait
		waitUntil(_pathFollowCompleted);

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
