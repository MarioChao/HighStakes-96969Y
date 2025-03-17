#pragma once

#include "Pas1-Lib/Planning/Trajectories/constraint.h"
#include "Pas1-Lib/Planning/Splines/curve-sampler.h"

#include <algorithm>
#include <vector>


namespace pas1_lib {
namespace planning {
namespace trajectories {


// ---------- Plan Point ----------

struct PlanPoint {
	PlanPoint(double time_seconds, double distance, std::vector<double> motion_dV_dT);

	void constrain(Constraint constraint);


	double time_seconds;
	double distance;
	std::vector<double> motion_dV_dT;
};


// ---------- Trajectory Planner ----------

class TrajectoryPlanner {
public:
	// Constructor; unit names are just for establishing consistency.
	TrajectoryPlanner(
		double distance_inches,
		std::vector<double> startMotion_dInches_dSec = {0, 5},
		std::vector<double> endMotion_dInches_dSec = {0, -5}
	);
	TrajectoryPlanner();

	TrajectoryPlanner &addConstraintSequence(ConstraintSequence constraints);
	TrajectoryPlanner &addConstraint_maxAcceleration(double maxAcceleration);
	TrajectoryPlanner &addConstraint_maxVelocity(double maxVelocity);
	TrajectoryPlanner &addConstraint_maxAngularVelocity(
		splines::CurveSampler curveSampler, double maxAngularVelocity,
		int distanceResolution
	);
	TrajectoryPlanner &addConstraint_maxCombinedVelocity(
		splines::CurveSampler curveSampler,
		double maxCombinedVelocity, double trackWidth,
		double minLinearVelocity,
		int distanceResolution
	);

	PlanPoint _getNextPlanPoint(PlanPoint node, double distanceStep);
	std::vector<PlanPoint> _forwardPass(double distanceStep);
	std::vector<PlanPoint> _backwardPass(double distanceStep);
	TrajectoryPlanner &calculateMotionProfile(int distanceResolution = 100);

	double getTotalTime();

	/// @param time_seconds Time in seconds
	/// @return {distance, motion_dV_dT}
	std::pair<double, std::vector<double>> getMotionAtTime(double time_seconds);

private:
	double distance;
	std::vector<double> startMotion;
	std::vector<double> endMotion;
	int distance_sign;

	std::vector<ConstraintSequence> constraintSequences;

	std::vector<PlanPoint> profilePoints;
};


}
}
}
