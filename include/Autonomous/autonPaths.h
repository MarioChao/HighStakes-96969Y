#pragma once

#include "Autonomous/autonFunctions.h"

#include "Pas1-Lib/Planning/Segments/cubic-spline.h"
#include "Pas1-Lib/Planning/Splines/curve-sampler.h"
#include "Pas1-Lib/Planning/Trajectories/trajectory-planner.h"
#include "Pas1-Lib/Planning/Trajectories/trajectory-constraint.h"

#include "Pas1-Lib/Chassis/Move/global-move-to.h"
#include "Pas1-Lib/Chassis/Move/local-move-by.h"
#include "Pas1-Lib/Chassis/Move/path-follow.h"

#include "Aespa-Lib/Karina-Data-Structures/linegular.h"
#include "Aespa-Lib/Winter-Utilities/angle.h"
#include "Aespa-Lib/Winter-Utilities/general.h"

#include "Utilities/fieldInfo.h"
#include "Utilities/debugFunctions.h"

#include "chassis-config.h"
#include "global-vars.h"
#include "main.h"

#include <string>


namespace autonpaths {
using namespace autonfunctions;
using namespace aespa_lib::units::literals;
using pas1_lib::chassis::base::Differential;
using namespace pas1_lib::chassis::move;
using pas1_lib::planning::trajectories::TrajectoryConstraint;
using aespa_lib::geometry::Polygon2D;
using pas1_lib::planning::trajectories::PolygonRegionConstraint;

// Build paths

namespace pathbuild {

void storeNewSplineProfile(
	std::string profileName,
	pas1_lib::planning::splines::SplineCurve spline, bool reverse, std::vector<TrajectoryConstraint *> constraints, double maxVel
);
void storeNewSplineProfile(
	std::string profileName,
	pas1_lib::planning::splines::SplineCurve spline, bool reverse = false, std::vector<TrajectoryConstraint *> constraints = {}
);

void runFollowSpline(Differential &chassis, std::string profileName, bool turnFirst = true);
void runFollowSpline(std::string profileName, bool turnFirst = true);

extern std::vector<std::vector<std::vector<aespa_lib::units::Length>>> linearPaths;
extern std::vector<double> linearMaxVelocity_pct;
extern std::vector<bool> linearWillReverse;

extern int linearIndex;

void clearLinear();
void pushNewLinear(std::vector<std::vector<aespa_lib::units::Length>> path, bool reverse = false, double maxVelocity_pct = 100);
void runFollowLinearYield();
}


// Combinations

namespace combination {
void grabGoalAt(double x_tiles, double y_tiles, double grabAtDistanceError = 0.15);
}


// Store profiles

void storeProfiles_test();
void storeProfiles_redUp();
void storeProfiles_redUpSafe();
void storeProfiles_blueUp();
void storeProfiles_blueUpSafe();
void storeProfiles_redDown();
void storeProfiles_redDownSafe();
void storeProfiles_redDownLBRush();
void storeProfiles_blueDown();
void storeProfiles_blueDownSafe();
void storeProfiles_blueDownLBRush();
void storeProfiles_redSoloAWP();
void storeProfiles_blueSoloAWP();
void storeProfiles_redSoloAWP2();
void storeProfiles_blueSoloAWP2();
void storeProfiles_skills();
void storeProfiles_loveShape();
void storeProfiles_fieldTour();


// Paths

void runTemplate();

void runAutonTest();
void odometryRadiusTest();
void runRushTest();

void runAutonRedUp();
void runAutonRedUpSafe();
void runAutonBlueUp();
void runAutonBlueUpSafe();

void runAutonRedDown();
void runAutonRedDownSafe();
void runAutonRedDownLBRush();
void runAutonBlueDown();
void runAutonBlueDownSafe();
void runAutonBlueDownLBRush();

void runRedSoloAWP();
void runBlueSoloAWP();
void runRedSoloAWP2();
void runBlueSoloAWP2();

void runAutonSkills();

void runLoveShape();
void runFieldTour();


// Configs

namespace configs {
bool willDoAllianceStake();
void setDoAllianceStake(bool state);
}

}
