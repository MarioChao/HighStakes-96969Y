#pragma once

#include "Autonomous/autonFunctions.h"
#include "GraphUtilities/uniformCubicSpline.h"
#include "GraphUtilities/curveSampler.h"
#include "GraphUtilities/trajectoryPlanner.h"

#include "Utilities/robotInfo.h"
#include "Utilities/debugFunctions.h"
#include "main.h"

namespace autonpaths {
	using namespace autonfunctions;
	using namespace autonfunctions::driveturn;
	using namespace botinfo;

	// Build paths

	namespace pathbuild {
		extern const double maxVel;
		extern const double maxAccel;
		extern const double maxDecel;

		extern std::vector<UniformCubicSpline> splines;
		extern std::vector<CurveSampler> splineSamplers;
		extern std::vector<TrajectoryPlanner> splineTrajectoryPlans;
		extern std::vector<bool> willReverse;

		extern int pathIndex;

		void clearSplines();
		void pushNewSpline(UniformCubicSpline spline, bool reverse = false, double maxVel = pathbuild::maxVel);
		void runFollowSpline();

		extern std::vector<std::vector<std::vector<double>>> linearPaths;
		extern std::vector<double> linearMaxVelocity_pct;
		extern std::vector<bool> linearWillReverse;

		extern int linearIndex;

		void clearLinear();
		void pushNewLinear(std::vector<std::vector<double>> path, bool reverse = false, double maxVelocity_pct = 100);
		void runFollowLinearYield();
	}


	// Combinations

	namespace combination {
		void grabGoalAt(double x_tiles, double y_tiles, double grabAtDistanceError = 0.15);
	}


	// Paths

	void runTemplate();

	void autonTest();
	void odometryRadiusTest();

	void runAutonRedUp();
	void runAutonRedUpSafe();
	void runAutonBlueUp();
	void runAutonBlueUpSafe();

	void runAutonRedDown();
	void runAutonBlueDown();
	void runAutonBlueDownSafe();

	void runRedSoloAWP();
	void runBlueSoloAWP();

	void runAutonSkills59();
	void runAutonSkillsNoWallStake();
	void runAutonSkillsLong();
	void runAutonSkills();

	void runAllianceWallStake();
	void runLoveShape();
	void runFieldTour();
}
