#include "Autonomous/autonPaths.h"

namespace {
	const double grabGoalVelocity_pct = 70;
}

namespace autonpaths { namespace combination {
	void grabGoalAt(double x_tiles, double y_tiles, double grabAtDistanceError) {
		turnToFace_tiles(x_tiles, y_tiles, true);
		async_driveTurnToFace_tiles(x_tiles, y_tiles, true, grabGoalVelocity_pct);
		waitUntil(_linearPathDistanceError < grabAtDistanceError);
		setGoalClampState(1);
		waitUntil(_isDriveTurnSettled);
	}
}}