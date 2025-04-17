#pragma once

#include "Pas1-Lib/Chassis/Base/differential.h"
#include "Pas1-Lib/Planning/Profiles/spline-profile.h"
#include "vex.h"


namespace pas1_lib {
namespace chassis {
namespace move {
namespace follow {


struct ramseteFollowPath_params {
	ramseteFollowPath_params(planning::profiles::SplineProfile *splineProfile)
		: splineProfile(splineProfile) {}

	planning::profiles::SplineProfile *splineProfile;
};

void ramseteFollowPath(base::Differential &chassis, ramseteFollowPath_params params, bool async);

extern timer _ramsetePathTimer;
extern double _ramseteFollowDistanceRemaining_tiles;
extern bool _isRamsetePathFollowCompleted;


}
}
}
}
