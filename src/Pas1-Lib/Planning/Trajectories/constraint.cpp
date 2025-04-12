#include "Pas1-Lib/Planning/Trajectories/constraint.h"

#include "Aespa-Lib/Winter-Utilities/general.h"
#include <cmath>


namespace pas1_lib {
namespace planning {
namespace trajectories {


// ---------- Constraint ----------

DistanceConstraint::DistanceConstraint(double distance, std::vector<double> maxMotion_dV_dT)
	: distance(distance), maxMotion_dV_dT(maxMotion_dV_dT) {}


// ---------- Constraint Sequence ----------

ConstraintSequence::ConstraintSequence(std::vector<DistanceConstraint> constraints, bool lerped)
	: constraints(constraints), lerped(lerped), isSorted(false) {}

ConstraintSequence::ConstraintSequence()
	: ConstraintSequence({}, false) {}

ConstraintSequence &ConstraintSequence::addConstraints(
	std::vector<std::pair<double, std::vector<double>>> constraints
) {
	for (std::pair<double, std::vector<double>> &constraint_raw : constraints) {
		DistanceConstraint constraint(constraint_raw.first, constraint_raw.second);
		this->constraints.push_back(constraint);
	}
	isSorted = false;
	return *this;
}

void ConstraintSequence::sort() {
	if (!isSorted) {
		std::sort(
			constraints.begin(), constraints.end(),
			[&](DistanceConstraint a, DistanceConstraint b) -> bool {return a.distance < b.distance;}
		);
		isSorted = true;
	}
}

DistanceConstraint ConstraintSequence::getConstraintAtDistance(double distance) {
	// Binary search for segment
	sort();
	if (constraints.empty()) return DistanceConstraint({ 0, {} });

	int bs_result = 0;
	int bs_l, bs_r, bs_m;
	bs_l = 0;
	bs_r = (int) constraints.size() - 1;
	while (bs_l <= bs_r) {
		bs_m = bs_l + (bs_r - bs_l) / 2;
		DistanceConstraint &constraint = constraints[bs_m];
		if (constraint.distance <= distance + 1e-4) {
			bs_result = bs_m;
			bs_l = bs_m + 1;
		} else {
			bs_r = bs_m - 1;
		}
	}

	// Get segment
	DistanceConstraint result = constraints[bs_result];

	// Lerped?
	if (lerped && bs_result + 1 < (int) constraints.size()) {
		// Get constraint bounds
		DistanceConstraint &c1 = constraints[bs_result];
		DistanceConstraint &c2 = constraints[bs_result + 1];

		// Linearly interpolate
		double lerp_t = (distance - c1.distance) / (c2.distance - c1.distance);
		result.distance = aespa_lib::genutil::lerp(c1.distance, c2.distance, lerp_t);
		for (int degree = 0; degree < (int) result.maxMotion_dV_dT.size(); degree++) {
			result.maxMotion_dV_dT[degree] = aespa_lib::genutil::lerp(
				c1.maxMotion_dV_dT[degree], c2.maxMotion_dV_dT[degree], lerp_t
			);
		}
	}

	// Return
	return result;
}

double getMinimumMotionAtDegree(std::vector<DistanceConstraint> constraints, int dV_dT_degree) {
	double result = -1;
	for (DistanceConstraint &constraint : constraints) {
		// Get motion value
		if (dV_dT_degree >= (int) constraint.maxMotion_dV_dT.size()) {
			continue;
		}
		double motionValue = constraint.maxMotion_dV_dT[dV_dT_degree];

		// Get minimum
		if (result < 0) result = motionValue;
		else result = std::fmin(result, motionValue);
	}
	return result;
}

DistanceConstraint getMinimumConstraint(std::vector<DistanceConstraint> constraints) {
	DistanceConstraint result = DistanceConstraint(0, {});
	for (DistanceConstraint &constraint : constraints) {
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

std::vector<DistanceConstraint> getConstraintsAtDistance(
	std::vector<ConstraintSequence> constraintSequences, double distance
) {
	std::vector<DistanceConstraint> result;
	for (ConstraintSequence &sequence : constraintSequences) {
		DistanceConstraint constraint = sequence.getConstraintAtDistance(distance);
		result.push_back(constraint);
	}

	// Return
	return result;
}

std::vector<DistanceConstraint> getConstraintsAtIndex(
	std::vector<ConstraintSequence> constraintSequences, int index
) {
	std::vector<DistanceConstraint> result;
	for (ConstraintSequence &sequence : constraintSequences) {
		if (index < (int) sequence.constraints.size()) {
			DistanceConstraint constraint = sequence.constraints[index];
			result.push_back(constraint);
		}
	}

	// Return
	return result;
}


}
}
}
