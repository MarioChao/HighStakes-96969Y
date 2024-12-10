#include "Autonomous/autonPaths.h"
#include "Simulation/robotSimulator.h"

namespace {
	double maxVel = 2.7;
	double maxAccel = 3;

	std::vector<UniformCubicSpline> splines;
	std::vector<CurveSampler> splineSamplers;
	std::vector<TrajectoryPlanner> splineTrajectoryPlans;

	void loadSkillsSplines() {
		if (splines.empty()) {
			// Alliance wall stake to mobile goal

			UniformCubicSpline spline = UniformCubicSpline::fromAutoTangent(cspline::CatmullRom, {
				{-0.64, 2.86}, {0.29, 2.92}, {1, 2.04}, {0.96, 0.02}
			});
			CurveSampler splineSampler = CurveSampler(spline)
				.calculateByResolution(spline.getTRange().second * 10);
			TrajectoryPlanner splineTrajectoryPlan = TrajectoryPlanner(splineSampler.getDistanceRange().second)
				.autoSetMotionConstraints(splineSampler, 0.5, maxVel, maxAccel, maxAccel)
				.calculateMotion();
			splines.push_back(spline);
			splineSamplers.push_back(splineSampler);
			splineTrajectoryPlans.push_back(splineTrajectoryPlan);

			// Redirect 1 ring to arm & score 6 rings
			spline = UniformCubicSpline::fromAutoTangent(cspline::CatmullRom, {
				{-0.17, 2.03}, {1, 2.03}, {1.98, 1.95}, {3.02, 1.15}, {3.95, 1.01},
				{3.1, 0.5}, {1.96, 1}, {0.48, 1.03}, {1.04, 0.49}, {2, 0.43}
			});
			splineSampler = CurveSampler(spline)
				.calculateByResolution(spline.getTRange().second * 10);
			splineTrajectoryPlan = TrajectoryPlanner(splineSampler.getDistanceRange().second)
				.autoSetMotionConstraints(splineSampler, 0.5, maxVel, maxAccel, maxAccel)
				.calculateMotion();
			splines.push_back(spline);
			splineSamplers.push_back(splineSampler);
			splineTrajectoryPlans.push_back(splineTrajectoryPlan);

			// Score on neutral wall stake
			spline = UniformCubicSpline::fromAutoTangent(cspline::CatmullRom, {
				{-0.44, 0.21}, {0.58, 0.49}, {2.78, 1.16}, {3, 0.52}, {3.02, -0.39}
			});
			splineSampler = CurveSampler(spline)
				.calculateByResolution(spline.getTRange().second * 10);
			splineTrajectoryPlan = TrajectoryPlanner(splineSampler.getDistanceRange().second)
				.autoSetMotionConstraints(splineSampler, 0.5, maxVel, maxAccel, maxAccel)
				.calculateMotion();
			splines.push_back(spline);
			splineSamplers.push_back(splineSampler);
			splineTrajectoryPlans.push_back(splineTrajectoryPlan);
		}
	}
}

/// @brief Run the skills autonomous.
void autonpaths::runAutonSkills() {
	/* Pre skills */

	// Timer
	timer autontimer;

	// Pre-process
	loadSkillsSplines();

	// Set position and rotation
	mainOdometry.setPosition(1.0, 3);
	mainOdometry.setLookAngle(-90.0);
	mainOdometry.printDebug();
	setRotation(-90.0);
	mainOdometry.restart();
	mainOdometry.printDebug();

	// Set config
	setDifferentialUseRelativeRotation(true);

	// Wait for arm reset
	waitUntil(isArmResetted());


	/* Score alliance wall stake and grab goal */

	// Score preload at alliance wall stake
	setArmStage(2);
	task::sleep(600);
	driveAndTurnDistanceTiles(0.5, -90.0, 100.0, 100.0, defaultMoveTilesErrorRange, 0.5);
	driveAndTurnDistanceTiles(-0.6, -90.0, 70.0, 100.0, defaultMoveTilesErrorRange, 0.5);
	setArmStage(1, 0.5);

	// Follow path
	setSplinePath(splines[0], splineTrajectoryPlans[0], splineSamplers[0]);
	followSplinePath(true);

	// Wait
	waitUntil(_pathFollowCompleted);

	// Grab goal
	setGoalClampState(1);
	wait(200, msec);


	/* Redirect 1 ring and score 6 rings */

	// Start intake
	turnToAngle(90.0);
	setIntakeState(1);
	setIntakeToArm(1);

	// Follow path
	setSplinePath(splines[1], splineTrajectoryPlans[1], splineSamplers[1]);
	followSplinePath();

	// Wait and remove redirect
	waitUntil(_trajectoryPlan.getMotionAtTime(_splinePathTimer.value())[0] >= 1.5);
	printf("No more redirect\n");
	setIntakeToArm(0);

	// Wait
	waitUntil(_pathFollowCompleted);

	// Place mobile goal in corner
	turnToAngle(70.0);
	setGoalClampState(0);
	driveAndTurnDistanceTiles(-0.60, 85.0, 80.0, 100.0, defaultMoveTilesErrorRange, 1.0);


	/* Score on neutral wall stake */

	// Raise arm
	setArmStage(3);

	// Follow path
	setSplinePath(splines[2], splineTrajectoryPlans[2], splineSamplers[2]);
	followSplinePath();

	// Wait
	waitUntil(_pathFollowCompleted);

	// Drive forward and score
	driveAndTurnDistanceTiles(0.5, 180.0, 100.0, 100.0, defaultMoveTilesErrorRange, 1.0);
	driveAndTurnDistanceTiles(-0.3, 180.0, 70.0, 100.0, defaultMoveTilesErrorRange, 0.5);


	/* Redirect 1 ring */

	turnToAngle(90.0);


	// Go to & face neutral wall stake
	setIntakeState(0);
	setIntakeToArm(0);
	turnToAngle(85.0);
	// setArmHangState(1);
	setArmStage(2);
	driveAndTurnDistanceTiles(2.46, 85.0, 100.0, 100.0, defaultMoveTilesErrorRange, 2.5);
	turnToAngleVelocity(180.0, 60.0);

	// Score ring at neutral wall stake
	driveAndTurnDistanceTiles(0.7, 180.0, 80.0, 100.0, defaultMoveTilesErrorRange, 1.5);
	// setArmHangState(0);
	setArmStage(0);
	driveAndTurnDistanceTiles(0.1, 180.0, 50.0, 100.0, defaultMoveTilesErrorRange, 0.5);
	task::sleep(200);

	/* Blue down */

	// Store 1 ring to arm
	turnToAngle(180.0);
	driveAndTurnDistanceTiles(-0.5, 180.0, 60.0, 100.0, defaultMoveTilesErrorRange, 1.5);
	turnToAngle(90.0);
	setIntakeToArm(1);
	setIntakeState(1);
	driveAndTurnDistanceTiles(1.0, 90.0, 80.0, 100.0, defaultMoveTilesErrorRange, 1.5);

	// Store 1 ring
	turnToAngle(0.0);
	setIntakeBottomState(1);
	setIntakeTopState(0);
	driveAndTurnDistanceTiles(1.0, 0.0, 80.0, 100.0, defaultMoveTilesErrorRange, 1.5);

	// Grab bottom mobile goal w/ blue ring
	turnToAngle(-90.0);
	driveAndTurnDistanceTiles(-0.9, -90.0, 100.0, 100.0, defaultMoveTilesErrorRange, 1.5);
	setGoalClampState(1, 0.6);
	driveAndTurnDistanceTiles(-0.6, -90.0, 60.0, 100.0, defaultMoveTilesErrorRange, 1.5);

	// Put mobile goal in corner
	turnToAngle(-15.0);
	setGoalClampState(0);
	driveAndTurnDistanceTiles(-1.55, -10.0, 90.0, 100.0, defaultMoveTilesErrorRange, 1.3);

	/* Blue up */

	// Go to middle mobile goal
	setIntakeState(0);
	driveAndTurnDistanceTiles(0.4, -20.0, 60.0, 100.0, defaultMoveTilesErrorRange, 1.5);
	turnToAngle(-20.0);
	driveAndTurnDistanceTiles(1.28, -20.0, 100.0, 100.0, defaultMoveTilesErrorRange, 1.5);

	// Grab middle mobile goal
	turnToAngle(-180.0);
	setGoalClampState(1, 0.7);
	driveAndTurnDistanceTiles(-0.80, -180.0, 60.0, 100.0, defaultMoveTilesErrorRange, 1.5);

	// Score 1 ring at blue wall stake
	// setArmHangState(1);
	setArmStage(2);
	turnToAngle(-280.0);
	driveAndTurnDistanceTiles(0.53, -270.0, 80.0, 100.0, defaultMoveTilesErrorRange, 1.0);
	// setArmHangState(0);
	setArmStage(0);
	task::sleep(400);
	driveAndTurnDistanceTiles(-0.3, -270.0, 60.0, 100.0, defaultMoveTilesErrorRange, 1.0);

	// Score 1 stored ring
	setIntakeState(1);

	// Take in 3 rings & score 
	turnToAngle(-270.0);
	setRotation(90.0);
	// First ring
	turnToAngle(-50.0);
	driveAndTurnDistanceTiles(1.86, -50.0, 90.0, 100.0, defaultMoveTilesErrorRange, 1.5);
	// Second ring
	turnToAngle(-135.0);
	setIntakeTopState(0);
	driveAndTurnDistanceTiles(1.0, -135.0, 90.0, 100.0, defaultMoveTilesErrorRange, 1.5);
	driveAndTurnDistanceTiles(-1.1, -135.0, 90.0, 100.0, defaultMoveTilesErrorRange, 1.5);
	setIntakeTopState(1);
	// Third ring
	turnToAngle(0);
	driveAndTurnDistanceTiles(1.03, 0, 90.0, 100.0, defaultMoveTilesErrorRange, 1.5);

	// Take in 2 rings in a row & score
	turnToAngle(90.0);
	driveAndTurnDistanceTiles(1.68, 90.0, 60.0, 100.0, defaultMoveTilesErrorRange, 1.0);

	// Take in 1 ring to arm
	turnToAngle(90.0);
	turnToAngle(0, halfRobotLengthIn * 1.0);
	setIntakeToArm(1);
	driveAndTurnDistanceTiles(0.5, 0, 70.0, 100.0, defaultMoveTilesErrorRange, 1.5);

	// Put mobile goal in corner
	turnToAngle(-100.0);
	setGoalClampState(0);
	driveAndTurnDistanceTiles(-0.7, -100.0, 80.0, 100.0, defaultMoveTilesErrorRange, 1.0);

	// Go to neutral wall stake & store 1 ring
	setIntakeToArm(0);
	setIntakeBottomState(1);
	setIntakeTopState(0);
	turnToAngle(-95);
	driveAndTurnDistanceTiles(2.46, -95, 80.0, 100.0, defaultMoveTilesErrorRange, 2.0);

	// Score ring at neutral wall stake
	turnToAngle(0);
	// setArmHangState(1);
	setArmStage(2);
	task::sleep(400);
	driveAndTurnDistanceTiles(1.0, 0.0, 80.0, 100.0, defaultMoveTilesErrorRange, 1.5);
	// setArmHangState(0);
	setArmStage(0);
	driveAndTurnDistanceTiles(0.1, 0.0, 50.0, 100.0, defaultMoveTilesErrorRange, 0.5);
	task::sleep(200);

	/* Red up */

	// Store 1 more ring
	driveAndTurnDistanceTiles(-0.6, 0.0, 60.0, 100.0, defaultMoveTilesErrorRange, 1.5);
	turnToAngle(-135.5);
	driveAndTurnDistanceTiles(1.43, -135.5, 70.0, 100.0, defaultMoveTilesErrorRange, 1.5);

	// Grab top mobile goal
	turnToAngle(-270.0);
	driveAndTurnDistanceTiles(-1.0, -270.0, 40.0, 100.0, defaultMoveTilesErrorRange, 1.5);
	setGoalClampState(1);

	// Score 2 stored rings
	setIntakeState(1);
	task::sleep(600);

	// Take in 4 rings & score
	setRotation(90.0);
	turnToAngle(0);
	driveAndTurnDistanceTiles(1.51, 0, 70.0, 100.0, defaultMoveTilesErrorRange, 1.5);
	turnToAngle(118.0);
	driveAndTurnDistanceTiles(1.10, 118.0, 70.0, 100.0, defaultMoveTilesErrorRange, 1.5);
	turnToAngle(270.0);
	driveAndTurnDistanceTiles(1.50, 270.0, 70.0, 100.0, defaultMoveTilesErrorRange, 1.5);

	// Place mobile goal in corner
	turnToAngle(170.0);
	setGoalClampState(0);
	driveAndTurnDistanceTiles(-0.70, 170.0, 60.0, 100.0, defaultMoveTilesErrorRange, 1.5);

	// Climb
	setIntakeState(0);
	turnToAngle(135.0);
	// setArmHangState(1);
	setArmStage(2);
	driveAndTurnDistanceTiles(2.35, 135.0, 60.0, 100.0, defaultMoveTilesErrorRange, 2.0);
	driveAndTurnDistanceTiles(1.0, 135.0, 30.0, 100.0, defaultMoveTilesErrorRange, 1.5);
	driveAndTurnDistanceTiles(-0.5, 135.0, 20.0, 100.0, defaultMoveTilesErrorRange, 1.5);
}
