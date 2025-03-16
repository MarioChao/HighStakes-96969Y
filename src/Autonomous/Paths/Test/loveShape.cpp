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

	// Set position and rotation
	mainOdometry.printDebug();
	mainOdometry.setPosition_scaled(1.5, 0.5);
	setRobotRotation(0);
	mainOdometry.printDebug();

	// Set config
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
		clearSplines();

		if (section == 1) {
			pushNewSpline(SplineCurve::fromAutoTangent_cubicSpline(CatmullRom, {
				{2.62, 0.09}, {1.52, 0.49}, {0.67, 1.35}, {1.03, 1.97}, {1.54, 1.8},
				{2.06, 1.95}, {2.49, 1.34}, {1.54, 0.48}, {0.48, 0.05},
			}));

			pushNewSpline(SplineCurve::fromAutoTangent_cubicSpline(CatmullRom, {
				{2.62, 0.09}, {1.52, 0.49}, {0.67, 1.35}, {1.03, 1.97}, {1.54, 1.8},
				{2.06, 1.95}, {2.49, 1.34}, {1.54, 0.48}, {0.48, 0.05},
			}), true);

			pushNewSpline(SplineCurve::fromAutoTangent_cubicSpline(CatmullRom, {
				{4.07, -0.01}, {3, 0.55}, {1.99, 1.99}, {0.92, 4.03}, {1.5, 5.2},
				{3.02, 4.68}, {4.52, 5.2}, {5.08, 4.01}, {4.03, 2.03}, {3.02, 0.57},
				{1.97, -0.07}
			}));
		}
	}

	void doAuton() {
		printf("master spark\n");

		// Follow path
		runFollowSpline();
		
		// Wait
		waitUntil(_pathFollowCompleted);
		printf("done\n");
		
		// Follow path again
		printf("<3\n");
		runFollowSpline();

		// Wait
		waitUntil(_pathFollowCompleted);
		printf("done\n");

		// Follow path
		printf("<3 big\n");
		// runFollowSpline();

		// Wait
		waitUntil(_pathFollowCompleted);
		printf("done\n");

		// Turn to 0
		turnToAngle(0);
	}
}
