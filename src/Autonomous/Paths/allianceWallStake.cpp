#include "Autonomous/autonPaths.h"

namespace {}

void autonpaths::runAllianceWallStake() {
	timer autontimer;
	setRobotRotation(-180.0);


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
