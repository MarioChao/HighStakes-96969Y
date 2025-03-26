#pragma once

#include "Pas1-Lib/Planning/Splines/spline-curve.h"
#include "Pas1-Lib/Planning/Splines/curve-sampler.h"
#include "Pas1-Lib/Planning/Trajectories/trajectory-planner.h"


namespace pas1_lib {
namespace planning {
namespace profiles {

struct SplineProfile {
	SplineProfile(
		splines::SplineCurve spline, splines::CurveSampler curveSampler,
		trajectories::TrajectoryPlanner trajectoryPlan, bool willReverse
	);

	splines::SplineCurve spline;
	splines::CurveSampler curveSampler;
	trajectories::TrajectoryPlanner trajectoryPlan;
	bool willReverse;
};

}
}
}
