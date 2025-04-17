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
	printf("field tour\n");

	// Follow path
	runFollowSpline("field tour");
	waitUntil(follow::_isRamsetePathFollowCompleted);
	printf("done\n");

	// Turn to 0
	local::turnToAngle(robotChassis, local::turnToAngle_params(0), false);
}


}
