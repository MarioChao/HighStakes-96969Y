#include "Autonomous/autonPaths.h"


namespace {
using pas1_lib::planning::splines::SplineCurve;
using pas1_lib::planning::splines::CurveSampler;
using pas1_lib::planning::trajectories::TrajectoryPlanner;
using pas1_lib::planning::profiles::SplineProfile;
}


namespace autonpaths {
namespace pathbuild {
// Build paths

// Default constants / constraints
const double maxVel_tilesPerSec = botinfo::maxV_tilesPerSec;
const double maxAccel = maxVel_tilesPerSec * 1.0;
const double maxDecel = maxVel_tilesPerSec * 1.0;

void clearSplines() {}

void storeNewSplineProfile(std::string profileName, SplineCurve spline, bool reverse, double maxVel) {
	if (splineProfile_storage.hasKey(profileName)) {
		// printf("Profile '%s' already exists!\n", profileName.c_str());
		return;
	}

	CurveSampler curveSampler = CurveSampler(spline)
		.calculateByResolution(spline.getTRange().second * 10);
	TrajectoryPlanner splineTrajectoryPlan = TrajectoryPlanner(
		curveSampler.getDistanceRange().second, botinfo::robotLengthIn / field::tileLengthIn,
		64
	)
		.setCurvatureFunction([&](double d) -> double {
		return spline.getCurvatureAt(curveSampler.distanceToParam(d));
	})
		.addCenterConstraint_maxMotion({ maxVel, maxAccel })
		.addTrackConstraint_maxMotion({ maxVel, maxAccel })
		.calculateMotionProfile();
	splineProfile_storage.store(profileName, SplineProfile(spline, curveSampler, splineTrajectoryPlan, reverse));
}

void runFollowSpline(std::string profileName) {
	SplineProfile *profile = splineProfile_storage.getStored(profileName).get();
	SplineCurve spline = profile->spline;
	CurveSampler curveSampler = profile->curveSampler;
	TrajectoryPlanner &motionProfile = profile->trajectoryPlan;
	bool isReversed = profile->willReverse;
	aespa_lib::datas::Linegular lg = spline.getLinegularAt(0, isReversed);
	
	autonfunctions::pid_diff::turnToAngle(aespa_lib::angle::swapFieldPolar_degrees(lg.getThetaPolarAngle_degrees()));
	autonfunctions::setSplinePath(spline, motionProfile, curveSampler);
	autonfunctions::followSplinePath(isReversed);
}


std::vector<std::vector<std::vector<double>>> linearPaths;
std::vector<double> linearMaxVelocity_pct;
std::vector<bool> linearWillReverse;

int linearIndex;

void clearLinear() {
	linearPaths.clear();
	linearMaxVelocity_pct.clear();
	linearWillReverse.clear();
	linearIndex = 0;
}

void pushNewLinear(std::vector<std::vector<double>> path, bool reverse, double maxVelocity_pct) {
	linearPaths.push_back(path);
	linearMaxVelocity_pct.push_back(maxVelocity_pct);
	linearWillReverse.push_back(reverse);
}

void runFollowLinearYield() {
	runLinearPIDPath(linearPaths[linearIndex], linearMaxVelocity_pct[linearIndex], linearWillReverse[linearIndex]);
	linearIndex++;
}
}
}
