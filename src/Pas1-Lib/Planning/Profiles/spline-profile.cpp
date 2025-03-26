#include "Pas1-Lib/Planning/Profiles/spline-profile.h"


namespace pas1_lib {
namespace planning {
namespace profiles {


SplineProfile::SplineProfile(
	splines::SplineCurve spline, splines::CurveSampler curveSampler,
	trajectories::TrajectoryPlanner trajectoryPlan, bool willReverse
)
	: spline(spline), curveSampler(curveSampler),
	trajectoryPlan(trajectoryPlan), willReverse(willReverse) {}


}
}
}
