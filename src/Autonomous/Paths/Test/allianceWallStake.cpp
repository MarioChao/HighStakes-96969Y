#include "Autonomous/autonPaths.h"

namespace {}

void autonpaths::runAllianceWallStake() {
	timer autontimer;
	setRobotRotation(-180.0);


	// Go back 1 tile
	local::driveAndTurn(robotChassis, local::driveAndTurn_params(-1.0, -180.0), false);


	// Score
	local::turnToAngle(robotChassis, local::turnToAngle_params(-90.0), false);
	setIntakeState(1);

	while (autontimer.value() < 12.0) {
		task::sleep(20);
	}


	setIntakeState(0);


	local::driveAndTurn(robotChassis, local::driveAndTurn_params(2.0, -90.0, 30.0, 100.0), false);
}
