#include "Autonomous/autonPaths.h"

namespace {
const double grabGoalVelocity_pct = 70;
}

namespace autonpaths {
namespace combination {

void grabGoalAt(double x_tiles, double y_tiles, double grabAtDistanceError) {
	global::turnToFace(robotChassis, global::turnToFace_params(x_tiles, y_tiles, true), false);
	global::driveToPoint(robotChassis, global::driveToPoint_params(x_tiles, y_tiles, 0, true, grabGoalVelocity_pct), true);
	waitUntil(global::_driveToPointDistanceError.tiles() < grabAtDistanceError);
	setGoalClampState(1);
	waitUntil(global::_isDriveToPointSettled);
}

}
}
