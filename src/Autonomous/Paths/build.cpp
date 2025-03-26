#include "Autonomous/autonPaths.h"


namespace {
using pas1_lib::planning::splines::SplineCurve;
using pas1_lib::planning::splines::CurveSampler;
using pas1_lib::planning::trajectories::TrajectoryPlanner;
}


namespace autonpaths {
namespace pathbuild {
// Build paths

// Default constants / constraints
const double maxVel_tilesPerSec = botinfo::maxV_tilesPerSec;
const double maxAccel = maxVel_tilesPerSec * 1.5;
const double maxDecel = maxVel_tilesPerSec * 1.5;
// const double maxJerk = maxAccel * 2.0;

// Global variables
std::vector<SplineCurve> splines;
std::vector<CurveSampler> splineSamplers;
std::vector<TrajectoryPlanner> splineTrajectoryPlans;
std::vector<bool> willReverse;

int pathIndex;

void clearSplines() {
	splines.clear();
	splineSamplers.clear();
	splineTrajectoryPlans.clear();
	willReverse.clear();
	pathIndex = 0;
}

void pushNewSpline(SplineCurve spline, bool reverse, double maxVel) {
	CurveSampler curveSampler = CurveSampler(spline)
		.calculateByResolution(spline.getTRange().second * 10);
	TrajectoryPlanner splineTrajectoryPlan = TrajectoryPlanner(
		curveSampler.getDistanceRange().second, robotLengthIn / field::tileLengthIn,
		64
	)
		.setCurvatureFunction([&](double d) -> double {
			return spline.getCurvatureAt(curveSampler.distanceToParam(d));
		})
		// .smoothenCurvature()
		.addCenterConstraint_maxMotion({maxVel, maxAccel})
		.addTrackConstraint_maxMotion({maxVel, maxAccel})
		.calculateMotionProfile();
	splines.push_back(spline);
	splineSamplers.push_back(curveSampler);
	splineTrajectoryPlans.push_back(splineTrajectoryPlan);
	willReverse.push_back(reverse);
}

void runFollowSpline() {
	aespa_lib::datas::Linegular lg = splines[pathIndex].getLinegularAt(0, willReverse[pathIndex]);
	autonfunctions::pid_diff::turnToAngle(aespa_lib::angle::swapFieldPolar_degrees(lg.getThetaPolarAngle_degrees()));
	autonfunctions::setSplinePath(splines[pathIndex], splineTrajectoryPlans[pathIndex], splineSamplers[pathIndex]);
	autonfunctions::followSplinePath(willReverse[pathIndex]);

	pathIndex++;
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
