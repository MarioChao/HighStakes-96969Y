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

		void pushNewSpline(UniformCubicSpline spline, bool reverse = false, double maxVel = pathbuild::maxVel);
		void loadSkillsSplines(int section);
		void runFollowSpline();
	}


	// Paths

	void autonTest();
	void odometryRadiusTest();

	void runAutonRedUp();
	void runAutonRedUpSafe();
	void runAutonRedDown();
	void runAutonRedDownOld();
	void runAutonBlueUp();
	void runAutonBlueUpSafe();
	void runAutonBlueDown();
	void runAutonBlueDownOld();
	void runAutonSkills();

	void runAllianceWallStake();
	void runLoveShape();
	void runFieldTour();
}
