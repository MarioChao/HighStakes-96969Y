#pragma once

#include <algorithm>
#include <vector>
#include <memory>
#include "Aespa-Lib/Karina-Data-Structures/linegular.h"


namespace pas1_lib {
namespace planning {
namespace trajectories {


// ---------- Constraint ----------

struct DistanceConstraint {
	DistanceConstraint(double distance, std::vector<double> maxMotion_dV_dT);


	double distance;
	std::vector<double> maxMotion_dV_dT;
};


// ---------- Constraint Sequence ----------

struct ConstraintSequence {
	ConstraintSequence(std::vector<DistanceConstraint> constraints, bool lerped);
	ConstraintSequence();

	ConstraintSequence &addConstraints(std::vector<std::pair<double, std::vector<double>>> constraints);

	void sort();
	DistanceConstraint getConstraintAtDistance(double distance);


	std::vector<DistanceConstraint> constraints;
	bool lerped;

	bool isSorted;
};

double getMinimumMotionAtDegree(std::vector<DistanceConstraint> constraints, int dV_dT_degree);
DistanceConstraint getMinimumConstraint(std::vector<DistanceConstraint> constraints);

std::vector<DistanceConstraint> getConstraintsAtDistance(
	std::vector<ConstraintSequence> constraintSequences, double distance
);

std::vector<DistanceConstraint> getConstraintsAtIndex(
	std::vector<ConstraintSequence> constraintSequences, int index
);


// ---------- Trajectory Constraint ----------

struct TrajectoryConstraint {
	virtual double calculateMaxVelocity(aespa_lib::datas::Linegular pose, double curvature, double velocity);
};

struct CentripetalAccelerationConstraint : public TrajectoryConstraint {
	CentripetalAccelerationConstraint(double maxCentripetalAcceleration);

	double calculateMaxVelocity(aespa_lib::datas::Linegular pose, double curvature, double velocity) override;


	double maxCentripetalAcceleration;
};

double getVelocity_trajectoryConstraints(std::vector<std::shared_ptr<TrajectoryConstraint>> constraints, aespa_lib::datas::Linegular pose, double curvature, double velocity);


}
}
}
