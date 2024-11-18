#pragma once

#include "Autonomous/autonFunctions.h"
#include "Mechanics/botIntake.h"
#include "Mechanics/botIntake2.h"
#include "Mechanics/swing.h"
#include "Utilities/robotInfo.h"
#include "Utilities/debugFunctions.h"
#include "main.h"

namespace autonpaths {
	using namespace autonfunctions;
	using namespace botinfo;

	void autonTest();

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
}
