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

void storeNewSplineProfile(std::string profileName, SplineCurve spline, bool reverse, double maxVel) {
	if (splineProfile_storage.hasKey(profileName)) {
		// printf("Profile '%s' already exists!\n", profileName.c_str());
		return;
	}

	CurveSampler curveSampler = CurveSampler(spline).calculateByResolution(spline.getTRange().second * 10);
	double totalDistance = curveSampler.getDistanceRange().second;
	double distanceStep = aespa_lib::genutil::clamp(totalDistance / 64, 0.077, 0.5);
	TrajectoryPlanner splineTrajectoryPlan = TrajectoryPlanner(curveSampler.getDistanceRange().second, botInfo.trackWidth_tiles, distanceStep)
		.setCurvatureFunction(
			[&](double d) -> double {
				return spline.getCurvatureAt(curveSampler.distanceToParam(d));
			},
			curveSampler.integerParamsToDistances()
		)
		.maxSmoothCurvature()
		.addCenterConstraint_maxMotion({ maxVel, botInfo.maxAccel_tilesPerSec2 * 0.85 })
		.addTrackConstraint_maxMotion({ maxVel, botInfo.maxAccel_tilesPerSec2 * 0.85 })
		.calculateMotionProfile();
	splineProfile_storage.store(profileName, SplineProfile(spline, curveSampler, splineTrajectoryPlan, reverse));
}

void storeNewSplineProfile(std::string profileName, pas1_lib::planning::splines::SplineCurve spline, bool reverse) {
	storeNewSplineProfile(profileName, spline, reverse, botInfo.maxVel_tilesPerSec);
}

void runFollowSpline(Differential &chassis, std::string profileName) {
	SplineProfile *splineProfile = splineProfile_storage.getStored(profileName).get();
	aespa_lib::datas::Linegular startPose = splineProfile->spline.getLinegularAt(0, splineProfile->willReverse);
	local::turnToAngle(chassis, local::turnToAngle_params(startPose.getRotation()), false);
	follow::followPath(chassis, follow::followPath_params(splineProfile), true);
}

void runFollowSpline(std::string profileName) {
	runFollowSpline(robotChassis, profileName);
}


std::vector<std::vector<std::vector<aespa_lib::units::Length>>> linearPaths;
std::vector<double> linearMaxVelocity_pct;
std::vector<bool> linearWillReverse;

int linearIndex;

void clearLinear() {
	linearPaths.clear();
	linearMaxVelocity_pct.clear();
	linearWillReverse.clear();
	linearIndex = 0;
}

void pushNewLinear(std::vector<std::vector<aespa_lib::units::Length>> path, bool reverse, double maxVelocity_pct) {
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
