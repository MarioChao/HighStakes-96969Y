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
