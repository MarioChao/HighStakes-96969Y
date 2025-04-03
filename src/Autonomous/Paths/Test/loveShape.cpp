#include "Autonomous/autonPaths.h"

namespace {
using namespace autonpaths;
using namespace autonpaths::pathbuild;

using pas1_lib::planning::splines::SplineCurve;
using pas1_lib::planning::splines::CurveSampler;
using namespace pas1_lib::planning::segments;

void loadPaths(int section);

void doAuton();
}

void autonpaths::runLoveShape() {
	/* Pre auton */

	// Timer
	_autonTimer.reset();

	// Set config
	setRobotPosition(1.5, 0.5);
	setRobotRotation(0);
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

	if (section == 1) {
	}
}

void doAuton() {
	printf("master spark\n");

	// Follow path
	runFollowSpline("big curvature 1");
	waitUntil(follow::_isPathFollowCompleted);
	printf("done\n");

	// Follow path
	runFollowSpline("big curvature 1br");
	waitUntil(follow::_isPathFollowCompleted);
	printf("done\n");

	// Follow path
	runFollowSpline("love 1");
	waitUntil(follow::_isPathFollowCompleted);
	printf("done\n");

	// Follow path again
	printf("<3\n");
	runFollowSpline("love 1r");
	waitUntil(follow::_isPathFollowCompleted);
	printf("done\n");

	// Follow path
	printf("<3 big\n");
	// runFollowSpline("love 2");
	waitUntil(follow::_isPathFollowCompleted);
	printf("done\n");

	// Turn to 0
	local::turnToAngle(robotChassis, local::turnToAngle_params(0), false);
}


}
