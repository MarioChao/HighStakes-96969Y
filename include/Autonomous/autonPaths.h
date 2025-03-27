#pragma once

#include "Autonomous/autonFunctions.h"

#include "Pas1-Lib/Planning/Segments/cubic-spline.h"
#include "Pas1-Lib/Planning/Splines/curve-sampler.h"
#include "Pas1-Lib/Planning/Trajectories/trajectory-planner.h"

#include "Pas1-Lib/Basic-Control/Move-Chassis/global-move-to.h"

#include "Aespa-Lib/Karina-Data-Structures/linegular.h"
#include "Aespa-Lib/Winter-Utilities/angle.h"
#include "Aespa-Lib/Winter-Utilities/general.h"

#include "Utilities/fieldInfo.h"
#include "Utilities/robotInfo.h"
#include "Utilities/debugFunctions.h"
#include "global-vars.h"
#include "main.h"

#include <string>


namespace autonpaths {
using namespace autonfunctions;
using namespace autonfunctions::pid_diff;
using namespace autonfunctions::driveturn;
using namespace botinfo;
using namespace pas1_lib::basic_control::move_chassis;

// Build paths

namespace pathbuild {
extern const double maxVel_tilesPerSec;
extern const double maxAccel;
extern const double maxDecel;

void clearSplines();
void storeNewSplineProfile(
	std::string profileName,
	pas1_lib::planning::splines::SplineCurve spline, bool reverse = false, double maxVel = pathbuild::maxVel_tilesPerSec
);
void runFollowSpline(std::string profileName);

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


// Store profiles

void storeSplineProfiles();


// Paths

void runTemplate();

void autonTest();
void odometryRadiusTest();

void runAutonRedUp();
void runAutonRedUpSafe();
void runAutonBlueUp();
void runAutonBlueUpSafe();

void runAutonRedDown();
void runAutonRedDownSafe();
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
