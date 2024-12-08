#include "Autonomous/autonPaths.h"

namespace {
	UniformCubicSpline loveSpline = UniformCubicSpline()
		.attachSegment(CubicSplineSegment(cspline::CatmullRom, {
			{2.62, 0.09}, {1.52, 0.49}, {0.67, 1.35}, {1.03, 1.97}
		}))
		.extendPoint({1.54, 1.8})
		.extendPoint({2.06, 1.95})
		.extendPoint({2.49, 1.34})
		.extendPoint({1.54, 0.48})
		.extendPoint({0.48, 0.05});
	CurveSampler loveSplineSampler = CurveSampler(loveSpline).calculateByResolution(loveSpline.getTRange().second * 7);
	TrajectoryPlanner loveSplineTrajectoryPlan = TrajectoryPlanner(loveSplineSampler.getDistanceRange().second)
		.addDesiredMotionConstraints(0, 2, 2, 2)
		.addDesiredMotionConstraints(1.2, 1.0, 2, 2)
		.addDesiredMotionConstraints(1.8, 0.5, 2, 2)
		.addDesiredMotionConstraints(3.2, 1.0, 2, 2)
		.addDesiredMotionConstraints(3.8, 2, 2, 2)
		.calculateMotion();
}

void autonpaths::runAllianceWallStake() {
	timer autontimer;
	setRotation(-180.0);


	// Go back 1 tile
	driveAndTurnDistanceTiles(-1.0, -180.0);


	// Score
	turnToAngle(-90.0);
	setIntakeState(1);

	while (autontimer.value() < 12.0) {
		task::sleep(20);
	}


	setIntakeState(0);


	driveAndTurnDistanceTiles(2.0, -90.0, 30.0, 100.0);
}

void autonpaths::runLoveShape() {
	printf("master spark\n");
	// Set robot position
	mainOdometry.setPosition(1.5, 0.5);
	mainOdometry.setLookAngle(0);
	setRotation(0);

	// Follow path
	setSplinePath(loveSpline, loveSplineTrajectoryPlan, loveSplineSampler);
	followSplinePath();

	// Wait
	waitUntil(_pathFollowCompleted);
	printf("done\n");

	// Follow path again
	printf("master spark\n");
	followSplinePath();

	// Wait
	waitUntil(_pathFollowCompleted);
	printf("done\n");

	// Turn to 0
	turnToAngle(0);
}
