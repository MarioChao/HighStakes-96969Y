#include "Autonomous/autonPaths.h"

namespace {
	double maxAccel = 1;

	UniformCubicSpline loveSpline;
	CurveSampler loveSplineSampler;
	TrajectoryPlanner loveSplineTrajectoryPlan;

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
