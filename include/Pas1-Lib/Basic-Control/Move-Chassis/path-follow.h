#pragma once

#include "Pas1-Lib/Basic-Control/Chassis/differential.h"
#include "Pas1-Lib/Planning/Profiles/spline-profile.h"
#include "vex.h"


namespace pas1_lib {
namespace basic_control {
namespace move_chassis {
namespace follow {


struct followPath_params {
	followPath_params(planning::profiles::SplineProfile *splineProfile)
		: splineProfile(splineProfile) {}

	planning::profiles::SplineProfile *splineProfile;
};

void followPath(chassis::Differential &chassis, followPath_params params, bool async);

extern timer _pathTimer;
extern double _pathFollowDistanceRemaining_tiles;
extern bool _isPathFollowCompleted;


}
}
}
}
