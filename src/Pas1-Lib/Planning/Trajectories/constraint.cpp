#include "Pas1-Lib/Planning/Trajectories/constraint.h"

#include "Aespa-Lib/Winter-Utilities/general.h"


namespace pas1_lib {
namespace planning {
namespace trajectories {


// ---------- Constraints ----------

Constraint::Constraint(double distance, std::vector<double> maxMotion_dV_dT)
	: distance(distance), maxMotion_dV_dT(maxMotion_dV_dT) {}

ConstraintSequence::ConstraintSequence(std::vector<Constraint> constraints)
	: constraints(constraints), isSorted(false) {}

ConstraintSequence &ConstraintSequence::addConstraints(
	std::vector<std::pair<double, std::vector<double>>> constraints
) {
	for (std::pair<double, std::vector<double>> &constraint_raw : constraints) {
		Constraint constraint(constraint_raw.first, constraint_raw.second);
		this->constraints.push_back(constraint);
	}
	isSorted = false;
	return *this;
}

void ConstraintSequence::sort() {
	if (!isSorted) {
		std::sort(
			constraints.begin(), constraints.end(),
			[&](Constraint a, Constraint b) -> bool {return a.distance < b.distance;}
		);
		isSorted = true;
	}
}

Constraint ConstraintSequence::getConstraintAtDistance(double distance) {
	// Binary search for segment
	sort();
	int bs_result = -1;
	int bs_l, bs_r, bs_m;
	bs_l = 0;
	bs_r = (int) constraints.size() - 1;
	while (bs_l <= bs_r) {
		bs_m = bs_l + (bs_r - bs_l) / 2;
		Constraint &constraint = constraints[bs_m];
		if (constraint.distance <= distance) {
			bs_result = bs_m;
			bs_l = bs_m + 1;
		} else {
			bs_r = bs_m - 1;
		}
	}
	if (bs_result == -1) bs_result = 0;

	// Return
	return constraints[bs_result];
}

Constraint getMinimumConstraint(std::vector<Constraint> constraints) {
	Constraint result = Constraint(0, {});
	for (Constraint &constraint : constraints) {
		for (int i = 0; i < (int) constraint.maxMotion_dV_dT.size(); i++) {
			// Get info
			double motionValue = constraint.maxMotion_dV_dT[i];

			// No comparison case
			if ((int) result.maxMotion_dV_dT.size() - 1 < i) {
				result.maxMotion_dV_dT.push_back(motionValue);
				continue;
			}

			// Get minimum
			if (motionValue < result.maxMotion_dV_dT[i]) {
				result.maxMotion_dV_dT[i] = motionValue;
			}
		}
	}

	// Return
	return result;
}

std::vector<Constraint> getConstraintsAtDistance(
	std::vector<ConstraintSequence> constraintSequences, double distance
) {
	std::vector<Constraint> result;
	for (ConstraintSequence &sequence : constraintSequences) {
		Constraint constraint = sequence.getConstraintAtDistance(distance);
		result.push_back(constraint);
	}

	// Return
	return result;
}


}
}
}
