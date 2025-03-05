#include "Autonomous/autonValues.h"

#include "Utilities/fieldInfo.h"
#include "Utilities/robotInfo.h"

namespace autonvals {
	const double defaultMoveTilesErrorRange = 0.04;
	const double defaultMoveWithInchesErrorRange = defaultMoveTilesErrorRange * field::tileLengthIn;
	const double defaultTurnAngleErrorRange = 5;

	const double scoreAllianceWallStakeVelocity_pct = 60;
	const double scoreNeutralWallStakeVelocity_pct = 80;
	const double rushGoalDeployDelay_msec = 250;
}
