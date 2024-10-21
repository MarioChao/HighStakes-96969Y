#pragma once

#include "Autonomous/autonFunctions.h"
#include "Mechanics/botIntake.h"
#include "Mechanics/swing.h"
#include "Utilities/robotInfo.h"
#include "Utilities/debugFunctions.h"
#include "main.h"

namespace autonpaths {
	using namespace autonfunctions;
	using namespace botinfo;

	void autonTest();

	void runAutonRedUpNew();
	void runAutonRedUp();
	void runAutonRedDown();
	void runAutonBlueUpNew();
	void runAutonBlueUp();
	void runAutonBlueDown();
	void runAutonSkills();
	void runAllianceWallStake();
}
