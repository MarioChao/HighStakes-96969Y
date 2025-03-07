#include "Autonomous/autonPaths.h"

namespace autonpaths { namespace pathbuild {
	// Build paths

	// Default constants / constraints
	const double maxVel_tilesPerSec = botinfo::maxV_tilesPerSec;
	const double maxAccel = maxVel_tilesPerSec * 1.1;
	const double maxDecel = maxVel_tilesPerSec * 1.1;

	// Global variables
	std::vector<UniformCubicSpline> splines;
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

	void pushNewSpline(UniformCubicSpline spline, bool reverse, double maxVel) {
		CurveSampler splineSampler = CurveSampler(spline)
			.calculateByResolution(spline.getTRange().second * 10);
		TrajectoryPlanner splineTrajectoryPlan = TrajectoryPlanner(splineSampler.getDistanceRange().second)
			.autoSetMotionConstraints(splineSampler, 0.3, maxVel, maxAccel, maxDecel)
			.calculateMotion();
		splines.push_back(spline);
		splineSamplers.push_back(splineSampler);
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

	void pushNewLinear(std::vector<std::vector<double>> path, bool reverse,  double maxVelocity_pct) {
		linearPaths.push_back(path);
		linearMaxVelocity_pct.push_back(maxVelocity_pct);
		linearWillReverse.push_back(reverse);
	}

	void runFollowLinearYield() {
		runLinearPIDPath(linearPaths[linearIndex], linearMaxVelocity_pct[linearIndex], linearWillReverse[linearIndex]);
		linearIndex++;
	}
}}
