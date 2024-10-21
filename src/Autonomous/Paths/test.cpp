#include "Autonomous/autonPaths.h"

void autonpaths::autonTest() {
	setRotation(0.0);

	driveAndTurnDistanceTiles(1.0, 0.0, 50.0, 100.0, defaultMoveTilesErrorRange, 6.0);
	// driveAndTurnDistanceTiles(1.0, 0.0, 100.0, 100.0, defaultMoveTilesErrorRange, 3.0);
	// driveAndTurnDistanceTiles(-1.0, 0.0, 100.0, 100.0, defaultMoveTilesErrorRange, 3.0);
	// driveAndTurnDistanceTiles(-1.0, 0.0, 100.0, 100.0, defaultMoveTilesErrorRange, 3.0);
	// driveAndTurnDistanceTiles(2.0, 0.0, 100.0, 100.0, defaultMoveTilesErrorRange, 3.0);
	// driveAndTurnDistanceTiles(-2.0, 0.0, 100.0, 100.0, defaultMoveTilesErrorRange, 3.0);

	// driveAndTurnDistanceTilesMotionProfile(1.0, 0.0, 100.0, 100.0, defaultMoveTilesErrorRange, 1.5);
	// driveAndTurnDistanceTilesMotionProfile(1.0, 0.0, 100.0, 100.0, defaultMoveTilesErrorRange, 1.5);
	// driveAndTurnDistanceTilesMotionProfile(-1.0, 0.0, 100.0, 100.0, defaultMoveTilesErrorRange, 1.5);
	// driveAndTurnDistanceTilesMotionProfile(-1.0, 0.0, 100.0, 100.0, defaultMoveTilesErrorRange, 1.5);
	// driveAndTurnDistanceTilesMotionProfile(2.0, 0.0, 100.0, 100.0, defaultMoveTilesErrorRange, 2.0);
	// driveAndTurnDistanceTilesMotionProfile(-2.0, 0.0, 100.0, 100.0, defaultMoveTilesErrorRange, 2.0);

	turnToAngle(50);
	turnToAngle(-50);
	turnToAngle(180);
	turnToAngle(-180);
	turnToAngle(0);
}
