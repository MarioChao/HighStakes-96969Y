#include "Autonomous/autonPaths.h"

namespace {
	double maxAccel = 1;

	UniformCubicSpline loveSpline;
	CurveSampler loveSplineSampler;
	TrajectoryPlanner loveSplineTrajectoryPlan;

	UniformCubicSpline bigLoveSpline;
	CurveSampler bigLoveSplineSampler;
	TrajectoryPlanner bigLoveSplineTrajectoryPlan;

	void loadLoveSpline() {
		if (loveSpline.getTRange().second == 0) {
			loveSpline = UniformCubicSpline()
				.attachSegment(CubicSplineSegment(cspline::CatmullRom, {
					{2.62, 0.09}, {1.52, 0.49}, {0.67, 1.35}, {1.03, 1.97}
				}))
				.extendPoints({
					{1.54, 1.8}, {2.06, 1.95}, {2.49, 1.34}, {1.54, 0.48}, {0.48, 0.05},
				});
			loveSplineSampler = CurveSampler(loveSpline)
				.calculateByResolution(loveSpline.getTRange().second * 7);
			loveSplineTrajectoryPlan = TrajectoryPlanner(loveSplineSampler.getDistanceRange().second)
				.addDesiredMotionConstraints(0, 1, maxAccel, maxAccel)
				.addDesiredMotionConstraints(1.2, 0.7, maxAccel, maxAccel)
				.addDesiredMotionConstraints(1.8, 0.4, maxAccel, maxAccel)
				.addDesiredMotionConstraints(3.2, 0.7, maxAccel, maxAccel)
				.addDesiredMotionConstraints(3.8, 1, maxAccel, maxAccel)
				.calculateMotion();

			bigLoveSpline = UniformCubicSpline()
				.attachSegment(CubicSplineSegment(cspline::CatmullRom, {
					{4.07, -0.01}, {3, 0.55}, {1.99, 1.99}, {0.92, 4.03}, {1.5, 5.2},
					{3.02, 4.68}, {4.52, 5.2}, {5.08, 4.01}, {4.03, 2.03}, {3.02, 0.57},
					{1.97, -0.07}
				}));
			bigLoveSplineSampler = CurveSampler(loveSpline)
				.calculateByResolution(loveSpline.getTRange().second * 7);
			bigLoveSplineTrajectoryPlan = TrajectoryPlanner(loveSplineSampler.getDistanceRange().second)
				.addDesiredMotionConstraints(0, 1, maxAccel, maxAccel)
				.addDesiredMotionConstraints(1.2, 0.7, maxAccel, maxAccel)
				.addDesiredMotionConstraints(1.8, 0.4, maxAccel, maxAccel)
				.addDesiredMotionConstraints(3.2, 0.7, maxAccel, maxAccel)
				.addDesiredMotionConstraints(3.8, 1, maxAccel, maxAccel)
				.calculateMotion();
		}
	}
}

void autonpaths::runLoveShape() {
	loadLoveSpline();
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
	printf("<3\n");
	followSplinePath(true);

	// Wait
	waitUntil(_pathFollowCompleted);
	printf("done\n");

	// Turn to 0
	turnToAngle(0);
}
