#include "Autonomous/autonValues.h"

#include "Utilities/fieldInfo.h"
#include "Utilities/robotInfo.h"

namespace autonvals {
	const double defaultMoveTilesErrorRange = 0.08;
	const double defaultMoveWithInchesErrorRange = defaultMoveTilesErrorRange * field::tileLengthIn;
	const double defaultTurnAngleErrorRange = 5;

	const double tilesPerSecond_to_pct = (
		// input: travel tiles per second
		(field::tileLengthIn / 1.0) // travel inches / sec
		* (1.0 / botinfo::driveWheelCircumIn) // wheel's rev / sec
		* (60.0 / 1.0) // wheel's rev / min
		* (botinfo::driveWheelMotorGearRatio / 1.0) // motor's rev / min
		* (1.0 / botinfo::chassisMotorRpm) // motor's pct [0-1]
		* (100.0 / 1.0) // motor's pct [0-100]
	);

	const double scoreAllianceWallStakeVelocity_pct = 60;
	const double scoreNeutralWallStakeVelocity_pct = 80;
	const double rushGoalDeployDelay_msec = 250;
}
