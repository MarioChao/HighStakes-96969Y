#include "Autonomous/autonPaths.h"

namespace {
	double maxAccel = 1;

	UniformCubicSpline spline = UniformCubicSpline()
		.attachSegment(CubicSplineSegment(cspline::CatmullRom, {
			{-0.02, -0.07}, {1.36, 0.64}, {2.4, 1.55}, {0.97, 2.99}
		}))
		.extendPoints({
			{0.42, 4.03}, {0.74, 5.28}, {2, 5.54}, {2.01, 3.98}, {3.02, 3},
			{4.03, 4.02}, {3.02, 4.85}, {3.02, 5.51}, {4.39, 5.49}, {4.67, 4.2},
			{5.55, 3.07}, {4.65, 1.77}, {5.49, 0.98}, {4.31, 0.42}, {4.02, 1.33},
			{3.15, 1.37}, {3, 0.48}, {3.02, -0.22},
		});
	CurveSampler splineSampler = CurveSampler(spline)
		.calculateByResolution(spline.getTRange().second * 7);
	TrajectoryPlanner splineTrajectoryPlan = TrajectoryPlanner(splineSampler.getDistanceRange().second)
		.addDesiredMotionConstraints(0, 0.7, maxAccel, maxAccel)
		.calculateMotion();
}

void autonpaths::runFieldTour() {
	// Set robot position
	mainOdometry.setPosition(1.5, 0.5);
	mainOdometry.setLookAngle(0);
	setRotation(0);

	// Follow path
	setSplinePath(spline, splineTrajectoryPlan, splineSampler);
	followSplinePath();

	// Wait
	waitUntil(_pathFollowCompleted);

	// Turn to 0
	turnToAngle(0);
}
