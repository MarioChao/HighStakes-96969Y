#pragma once

#include "Pas1-Lib/Planning/Trajectories/constraint.h"
#include "Pas1-Lib/Planning/Trajectories/curvature.h"

#include <algorithm>
#include <vector>
#include <functional>


namespace pas1_lib {
namespace planning {
namespace trajectories {


// ---------- Plan Point ----------

struct PlanPoint {
	PlanPoint(double time_seconds, double distance, std::vector<double> motion_dV_dT);

	PlanPoint &constrain(Constraint constraint);
	PlanPoint &maximizeLastDegree(Constraint constraint);


	double time_seconds;
	double distance;
	std::vector<double> motion_dV_dT;
};

double getTimeStepFromDistanceStep(PlanPoint node, double distanceStep);


// ---------- Trajectory Planner ----------

class TrajectoryPlanner {
public:
	// Constructor; unit names are just for establishing consistency.
	TrajectoryPlanner(
		double distance_inches,
		double trackWidth_inches,
		int distanceResolution,
		std::vector<double> startMotion_dInches_dSec,
		std::vector<double> endMotion_dInches_dSec
	);
	TrajectoryPlanner(
		double distance_inches, double trackWidth_inches, int distanceResolution
	);
	TrajectoryPlanner(double distance_inches, double trackWidth_inches);
	TrajectoryPlanner(double distance_inches);
	TrajectoryPlanner();

	TrajectoryPlanner &setCurvatureFunction(std::function<double(double)> distanceToCurvature_function);

	// alpha should change with distanceResolution
	TrajectoryPlanner &smoothenCurvature(double alpha = 0.7);
	double getCurvatureAtDistance(double distance);

	TrajectoryPlanner &addConstraintSequence(ConstraintSequence constraints);

	/// @param maxMotion_dV_dT Recommend only up to 3 degrees (jerk/jolt), and not too small.
	TrajectoryPlanner &addConstraint_maxMotion(std::vector<double> maxMotion_dV_dT);
	TrajectoryPlanner &addConstraint_maxAngularMotion(std::vector<double> maxAngularMotion);

	PlanPoint _getNextPlanPoint(PlanPoint node, double distanceStep);
	std::vector<PlanPoint> _forwardPass(double distanceStep);
	std::vector<PlanPoint> _backwardPass(double distanceStep);
	std::pair<ConstraintSequence, ConstraintSequence> constraintSequencesFromPlanPoints(
		std::vector<PlanPoint> nodes
	);
	TrajectoryPlanner &calculateMotionProfile();

	double getTotalTime();

	/// @param time_seconds Time in seconds
	/// @return {distance, motion_dV_dT}
	std::pair<double, std::vector<double>> getMotionAtTime(double time_seconds);

private:
	double distance;
	double trackWidth;
	int distance_sign;

	int distanceResolution;

	std::vector<double> startMotion;
	std::vector<double> endMotion;

	CurvatureSequence curvatureSequence;

	std::vector<ConstraintSequence> constraintSequences;
	std::vector<ConstraintSequence> center_constraintSequences;
	std::vector<ConstraintSequence> track_constraintSequences;

	std::vector<PlanPoint> profilePoints;
};


}
}
}
