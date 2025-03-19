#pragma once

#include <algorithm>
#include <vector>


namespace pas1_lib {
namespace planning {
namespace trajectories {


// ---------- Constraints ----------

struct Constraint {
	Constraint(double distance, std::vector<double> maxMotion_dV_dT);


	double distance;
	std::vector<double> maxMotion_dV_dT;
};

struct ConstraintSequence {
	ConstraintSequence(std::vector<Constraint> constraints, bool lerped);
	ConstraintSequence();

	ConstraintSequence &addConstraints(std::vector<std::pair<double, std::vector<double>>> constraints);

	void sort();
	Constraint getConstraintAtDistance(double distance);


	std::vector<Constraint> constraints;
	bool lerped;

	bool isSorted;
};

Constraint getMinimumConstraint(std::vector<Constraint> constraints);

std::vector<Constraint> getConstraintsAtDistance(
	std::vector<ConstraintSequence> constraintSequences, double distance
);


}
}
}
