#pragma once

#include "Pas1-Lib/Planning/Trajectories/constraint.h"
#include "Pas1-Lib/Planning/Trajectories/trajectory-constraint.h"
#include "Pas1-Lib/Planning/Trajectories/curvature.h"
#include "Pas1-Lib/Planning/Splines/curve-sampler.h"

#include <algorithm>
#include <vector>
#include <functional>
#include <memory>


namespace pas1_lib {
namespace planning {
namespace trajectories {


// ---------- Plan Point ----------

struct PlanPoint {
	PlanPoint(double time_seconds, double distance, std::vector<double> motion_dV_dT);

	PlanPoint &constrain(DistanceConstraint constraint);
	PlanPoint &maximizeLastDegree(DistanceConstraint constraint);
	PlanPoint &maximizeNthDegree(
		DistanceConstraint constraint, int dV_dT_degree,
		DistanceConstraint target_rawConstraint, bool maximizeLowerDegrees = false
	);


	double time_seconds;
	double distance;
	std::vector<double> motion_dV_dT;
};

double getTimeStepFromDistanceStep(PlanPoint node, double distanceStep);
ConstraintSequence planPoints_to_rawConstraintSequence(std::vector<PlanPoint> nodes);


// ---------- Trajectory Planner ----------

class TrajectoryPlanner {
public:
	// Constructor; unit names are just for establishing consistency.
	TrajectoryPlanner(
		double distance_inches,
		double trackWidth_inches,
		double planPoint_distanceStep,
		std::vector<double> startMotion_dInches_dSec,
		std::vector<double> endMotion_dInches_dSec
	);
	TrajectoryPlanner(
		double distance_inches, double trackWidth_inches, double planPoint_distanceStep
	);
	TrajectoryPlanner(double distance_inches, double trackWidth_inches);
	TrajectoryPlanner(double distance_inches);
	TrajectoryPlanner();


	// Curvature

	TrajectoryPlanner &setCurvatureFunction(
		std::function<double(double)> distanceToCurvature_function,
		std::vector<double> specificDistances = {}
	);

	TrajectoryPlanner &maxSmoothCurvature(double epsilon = 1e10);
	double getCurvatureAtDistance(double distance);


	// Spline

	TrajectoryPlanner &setSpline(splines::CurveSampler *curveSampler);
	aespa_lib::datas::Linegular getLinegularAtDistance(double distance);


	// Constriants

	TrajectoryPlanner &addCenterConstraintSequence(ConstraintSequence constraints);
	TrajectoryPlanner &addTrackConstraintSequence(ConstraintSequence constraints);
	TrajectoryPlanner &addCenterConstraints(std::vector<std::pair<double, std::vector<double>>> constraints);
	TrajectoryPlanner &addTrackConstraints(std::vector<std::pair<double, std::vector<double>>> constraints);

	// For now, only use up to dA/dT
	TrajectoryPlanner &addCenterConstraint_maxMotion(std::vector<double> maxMotion_dV_dT);
	// For now, only use up to dA/dT
	TrajectoryPlanner &addTrackConstraint_maxMotion(std::vector<double> maxMotion_dV_dT);

	TrajectoryPlanner &addCenterConstraint_maxCentripetalAcceleration(double maxCentripetalAceleration);
	TrajectoryPlanner &addCenterTrajectoryConstraints(std::vector<TrajectoryConstraint *> constraints);


	// Calculation

	PlanPoint _getNextPlanPoint(
		PlanPoint originalNode,
		double distanceStep,
		int dV_dT_degree
	);
	std::vector<PlanPoint> _forwardPass(int dV_dT_degree);
	std::vector<PlanPoint> _backwardPass(int dV_dT_degree);
	std::pair<ConstraintSequence, ConstraintSequence> constraintSequencesFromPlanPoints(
		std::vector<PlanPoint> nodes
	);
	TrajectoryPlanner &calculateMotionProfile();

	double getTotalTime();

	/// @param time_seconds Time in seconds
	/// @return {distance, motion_dV_dT}
	std::pair<double, std::vector<double>> getMotionAtTime(double time_seconds);

private:
	double totalDistance;
	double trackWidth;
	int distance_sign;

	double planPoint_distanceStep;

	std::vector<double> startMotion;
	std::vector<double> endMotion;

	CurvatureSequence curvatureSequence;
	splines::CurveSampler *curveSampler;

	std::vector<ConstraintSequence> center_constraintSequences;
	std::vector<ConstraintSequence> track_constraintSequences;
	std::vector<std::shared_ptr<TrajectoryConstraint>> center_trajectoryConstraints;

	std::vector<double> planPoint_distances;
	ConstraintSequence planPoint_rawSequence;
	std::vector<PlanPoint> profilePoints;
};


}
}
}
