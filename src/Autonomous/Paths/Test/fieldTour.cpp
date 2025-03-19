#include "Autonomous/autonPaths.h"


namespace {
using namespace autonpaths;
using namespace autonpaths::pathbuild;

using pas1_lib::planning::splines::SplineCurve;
using pas1_lib::planning::splines::CurveSampler;
using namespace pas1_lib::planning::segments;
using pas1_lib::planning::trajectories::TrajectoryPlanner;

void loadPaths(int section);

void doAuton();
}

void autonpaths::runFieldTour() {
	/* Pre auton */

	// Timer
	_autonTimer.reset();

	// Set position and rotation
	mainOdometry.printDebug();
	mainOdometry.setPosition_scaled(1.5, 0.5);
	setRobotRotation(0);
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
		pushNewSpline(SplineCurve::fromAutoTangent_cubicSpline(CatmullRom, {
			{-0.02, -0.07}, {1.36, 0.64}, {2.4, 1.55}, {0.97, 2.99}, {0.42, 4.03},
			{0.74, 5.28}, {2, 5.54}, {2.01, 3.98}, {3.02, 3}, {4.03, 4.02},
			{3.02, 4.85}, {3.02, 5.51}, {4.39, 5.49}, {4.67, 4.2}, {5.55, 3.07},
			{4.65, 1.77}, {5.49, 0.98}, {4.31, 0.42}, {4.02, 1.33}, {3.15, 1.37},
			{3, 0.48}, {3.02, -0.22},
			}));
	}
}

void doAuton() {
	printf("field tour\n");

	// Follow path
	runFollowSpline();

	// Wait
	waitUntil(_pathFollowCompleted);
	printf("done\n");

	// Turn to 0
	turnToAngle(0);
}


}
