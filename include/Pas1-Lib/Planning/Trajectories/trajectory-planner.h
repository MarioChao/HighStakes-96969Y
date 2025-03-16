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
	// Constructor; unitless, names are just for establishing consistency.
	TrajectoryPlanner(
		double distance_inches,
		double startVelocity_inchesPerSecond = 0,
		double endVelocity_inchesPerSecond = 0
	);
	TrajectoryPlanner();

	TrajectoryPlanner &addConstraintSequence(ConstraintSequence constraints);
	TrajectoryPlanner &addConstraint_maxAcceleration(double maxAcceleration);
	TrajectoryPlanner &addConstraint_maxVelocity(double maxVelocity);
	TrajectoryPlanner &addConstraint_maxAngularVelocity(
		splines::CurveSampler curveSampler, double maxAngularVelocity,
		int distanceResolution
	);

	PlanPoint _getNextPlanPoint(PlanPoint node, double timeStep_seconds);
	std::vector<PlanPoint> _forwardPass(double timeStep_seconds);
	std::vector<PlanPoint> _backwardPass(double timeStep_seconds);
	TrajectoryPlanner &calculateMotionProfile(double timeStep_seconds = 0.05);

	double getTotalTime();

	/// @param time_seconds Time in seconds
	/// @return {distance, motion_dV_dT}
	std::pair<double, std::vector<double>> getMotionAtTime(double time_seconds);

private:
	double distance;
	double startVelocity;
	double endVelocity;
	int distance_sign;

	std::vector<ConstraintSequence> constraintSequences;

	std::vector<PlanPoint> profilePoints;
};


}
}
}
