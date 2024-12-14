#include "Autonomous/autonPaths.h"

void autonpaths::autonTest() {
	setRobotRotation(0.0);

	// driveAndTurnDistanceTiles(1.0, 0.0, 50.0, 100.0, 6.0);
	// driveAndTurnDistanceTiles(1.0, 0.0, 100.0, 100.0, 3.0);
	// driveAndTurnDistanceTiles(-1.0, 0.0, 100.0, 100.0, 3.0);
	// driveAndTurnDistanceTiles(-1.0, 0.0, 100.0, 100.0, 3.0);
	// driveAndTurnDistanceTiles(2.0, 0.0, 100.0, 100.0, 3.0);
	// driveAndTurnDistanceTiles(-2.0, 0.0, 100.0, 100.0, 3.0);

	// turnToAngle(45);
	// task::sleep(200);
	// turnToAngleVelocity(-45, 15);
	// task::sleep(200);
	// turnToAngleVelocity(90, 30);
	// task::sleep(200);
	// turnToAngleVelocity(-90, 60);
	// task::sleep(200);
	// turnToAngle(180);
	// task::sleep(200);
	// turnToAngle(-180);
	// task::sleep(200);
	// turnToAngle(450);
	// task::sleep(200);
	// turnToAngle(-450, 0.0, defaultTurnAngleErrorRange, 3.5);
	// task::sleep(200);
	// turnToAngle(0);

	setDifferentialUseRelativeRotation(true);

	mainOdometry.setPosition(0, 0);
	setRobotRotation(0);

	runLinearPIDPath({
		{0, 1}, {1, 1}, {1, 0}, {0, 0}
	}, 100);
	turnToAngle(0);
}

void autonpaths::odometryRadiusTest() {
	setRobotRotation(0.0);

	wait(100, msec);

	printf("Clockwise\n");

	mainOdometry.printDebug();
	turnToAngleVelocity(360.0 * 10.0, 30.0, 0.0, 40.0);
	mainOdometry.printDebug();

	wait(1, sec);

	setRobotRotation(0.0);

	wait(100, msec);

	printf("Counter clockwise\n");

	mainOdometry.printDebug();
	turnToAngleVelocity(-360.0 * 10.0, 30.0, 0.0, 40.0);
	mainOdometry.printDebug();
}
