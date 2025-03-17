#include "Pas1-Lib/Planning/Trajectories/trajectory-planner.h"

#include "Aespa-Lib/Winter-Utilities/general.h"
#include <stdio.h>


namespace pas1_lib {
namespace planning {
namespace trajectories {


// ---------- Plan Point ----------

PlanPoint::PlanPoint(double time_seconds, double distance, std::vector<double> motion_dV_dT)
	: time_seconds(time_seconds), distance(distance),
	motion_dV_dT(motion_dV_dT) {}

void PlanPoint::constrain(Constraint constraint) {
	for (int i = 0; i < (int) motion_dV_dT.size(); i++) {
		// Validate constraint
		if (i >= (int) constraint.maxMotion_dV_dT.size()) {
			break;
		}

		// Constrain
		double maxValue = constraint.maxMotion_dV_dT[i];
		double currentValue = motion_dV_dT[i];
		motion_dV_dT[i] = aespa_lib::genutil::clamp(currentValue, -maxValue, maxValue);
	}
}


// ---------- Trajectory Planner ----------

TrajectoryPlanner::TrajectoryPlanner(
	double distance_inches,
	std::vector<double> startMotion_dInches_dSec,
	std::vector<double> endMotion_dInches_dSec
)
	:
	startMotion(startMotion_dInches_dSec),
	endMotion(endMotion_dInches_dSec),
	constraintSequences({}) {
	distance = fabs(distance_inches);
	distance_sign = aespa_lib::genutil::signum(distance_inches);
};

TrajectoryPlanner::TrajectoryPlanner()
	: TrajectoryPlanner(0) {};

TrajectoryPlanner &TrajectoryPlanner::addConstraintSequence(ConstraintSequence constraints) {
	constraintSequences.push_back(constraints);
	return *this;
}

TrajectoryPlanner &TrajectoryPlanner::addConstraint_maxAcceleration(double maxAcceleration) {
	addConstraintSequence(
		ConstraintSequence({})
		.addConstraints({ {0, {1e8, maxAcceleration}} })
	);
	return *this;
}

TrajectoryPlanner &TrajectoryPlanner::addConstraint_maxVelocity(double maxVelocity) {
	addConstraintSequence(
		ConstraintSequence({})
		.addConstraints({ {0, {maxVelocity}} })
	);
	return *this;
}

TrajectoryPlanner &TrajectoryPlanner::addConstraint_maxAngularVelocity(
	splines::CurveSampler curveSampler, double maxAngularVelocity,
	int distanceResolution
) {
	splines::SplineCurve splineCurve = curveSampler.getSpline();
	ConstraintSequence constraintSequence({});
	for (int resolutionI = 0; resolutionI <= distanceResolution; resolutionI++) {
		// Get distance
		double x = aespa_lib::genutil::rangeMap(resolutionI, 0, distanceResolution, 0, distance);

		// Get curvature
		double k = splineCurve.getCurvatureAt(curveSampler.distanceToParam(x));
		// printf("x: %.3f, k: %.3f\n", x, k);
		
		// Max linear velocity = max angular velocity / curvature
		double maxVelocity;
		if (fabs(k) < 1e-5) maxVelocity = 1e8;
		else maxVelocity = maxAngularVelocity / fabs(k);

		// Add constraint
		constraintSequence.addConstraints({
			{x, {maxVelocity}}
		});
	}

	// Add sequence
	addConstraintSequence(constraintSequence);

	return *this;
}

TrajectoryPlanner &TrajectoryPlanner::addConstraint_maxCombinedVelocity(
	splines::CurveSampler curveSampler,
	double maxCombinedVelocity, double trackWidth,
	double minLinearVelocity,
	int distanceResolution
) {
	splines::SplineCurve splineCurve = curveSampler.getSpline();
	ConstraintSequence constraintSequence({});
	for (int resolutionI = 0; resolutionI <= distanceResolution; resolutionI++) {
		// Get distance
		double x = aespa_lib::genutil::rangeMap(resolutionI, 0, distanceResolution, 0, distance);
		
		// Get curvature
		double k = splineCurve.getCurvatureAt(curveSampler.distanceToParam(x));

		// Equation:
		// maxLinearVelocity + maxLinearVelocity * curvature * trackWidth / 2 ≤ maxCombinedVelocity
		// -> maxLinearVelocity ≤ maxCombinedVelocity / (1 + curvature * trackWidth / 2)

		double maxLinearVelocity = maxCombinedVelocity / (1 + fabs(k) * trackWidth / 2);

		if (false) {
			// From https://github.com/baylessj/robotsquiggles/blob/main/main/src/tankmodel.cpp
			
			// Try driving at full speed
			double maxAngularVelocity = maxCombinedVelocity * fabs(k);
			double leftVelocity = maxCombinedVelocity - maxAngularVelocity * trackWidth / 2;
			double rightVelocity = maxCombinedVelocity + maxAngularVelocity * trackWidth / 2;

			// Resolve overshoot
			double scaleFactor = aespa_lib::genutil::getScaleFactor(maxCombinedVelocity, {leftVelocity, rightVelocity});
			leftVelocity *= scaleFactor;
			rightVelocity *= scaleFactor;

			// Compute linear velocity
			maxLinearVelocity = (leftVelocity + rightVelocity) / 2;
		}

		// Add constraint
		constraintSequence.addConstraints({
			{x, {maxLinearVelocity}}
		});
	}

	// Add sequence
	addConstraintSequence(constraintSequence);

	return *this;
}

PlanPoint TrajectoryPlanner::_getNextPlanPoint(PlanPoint node, double distanceStep) {
	// Minimum constrain
	double oldDistance = node.distance;
	std::vector<Constraint> constraints = getConstraintsAtDistance(
		constraintSequences, oldDistance
	);
	Constraint minIsolatedConstraint = getMinimumConstraint(constraints);
	node.constrain(minIsolatedConstraint);

	// printf("%.3f ", oldDistance);
	// for (double a : node.motion_dV_dT) {
	// 	printf("%.3f ", a);
	// }
	// printf("\n");

	// Binary search for time step
	double timeStep_seconds = [&]() -> double {
		double bs_l, bs_r, bs_m;
		int bs_trials = 0;
		double bs_error = 1e9;
		bs_l = -10;
		bs_r = 10;
		while (fabs(bs_error) >= 1e-8 && bs_trials <= 100) {
			bs_m = bs_l + (bs_r - bs_l) / 2;
			double bs_distanceStep = aespa_lib::genutil::integratePolynomial(node.motion_dV_dT, bs_m).first;
			bs_error = distanceStep - bs_distanceStep;
			if (bs_distanceStep < distanceStep) {
				bs_l = bs_m;
			} else {
				bs_r = bs_m;
			}
			bs_trials++;
		}
		return bs_m;
	}();

	// Integrate
	auto integral = aespa_lib::genutil::integratePolynomial(
		node.motion_dV_dT, timeStep_seconds
	);
	double newDistance = oldDistance + integral.first;
	double time_seconds = node.time_seconds + timeStep_seconds;
	PlanPoint newNode(time_seconds, newDistance, integral.second);

	// Minimum constrain
	std::vector<Constraint> constraints2 = getConstraintsAtDistance(
		constraintSequences, newDistance
	);
	Constraint minIsolatedConstraint2 = getMinimumConstraint(constraints2);
	newNode.constrain(minIsolatedConstraint2);

	// Derivative constrain
	for (int i = 1; i < (int) newNode.motion_dV_dT.size(); i++) {
		double deriv = fabs((newNode.motion_dV_dT[i - 1] - node.motion_dV_dT[i - 1]) / timeStep_seconds);
		newNode.motion_dV_dT[i] = aespa_lib::genutil::clamp(newNode.motion_dV_dT[i], -deriv, deriv);
	}

	// printf("%.3f ", newDistance);
	// for (double a : newNode.motion_dV_dT) {
	// 	printf("%.3f ", a);
	// }
	// printf("\n");

	// TODO: remove sharp deceleration

	// Return
	return newNode;
}

std::vector<PlanPoint> TrajectoryPlanner::_forwardPass(double distanceStep) {
	std::vector<PlanPoint> planningPoints;
	planningPoints.push_back(PlanPoint(0, 0, startMotion));

	while (true) {
		// Get info
		PlanPoint lastNode = planningPoints.back();

		// Get planning point
		PlanPoint newNode = _getNextPlanPoint(lastNode, distanceStep);

		// Validate total distance not exceeded
		if (newNode.distance >= distance) {
			break;
		}

		// Push planning point
		planningPoints.push_back(newNode);
	}
	planningPoints.push_back(_getNextPlanPoint(planningPoints.back(), distance - planningPoints.back().distance));

	return planningPoints;
}

std::vector<PlanPoint> TrajectoryPlanner::_backwardPass(double distanceStep) {
	std::vector<PlanPoint> planningPoints;
	planningPoints.push_back(PlanPoint(0, distance, endMotion));

	while (true) {
		// Get info
		PlanPoint lastNode = planningPoints.back();

		// Get planning point
		PlanPoint newNode = _getNextPlanPoint(lastNode, -distanceStep);

		// Validate total distance not exceeded
		if (newNode.distance <= 0) {
			break;
		}

		// Push planning point
		planningPoints.push_back(newNode);
	}

	// Flip derivatives
	for (PlanPoint &node : planningPoints) {
		for (int i = 1; i < (int) node.motion_dV_dT.size(); i++) {
			node.motion_dV_dT[i] *= -1;
		}
	}

	// Reverse
	std::reverse(planningPoints.begin(), planningPoints.end());

	return planningPoints;
}

TrajectoryPlanner &TrajectoryPlanner::calculateMotionProfile(int distanceResolution) {
	double distanceStep = distance / (double) distanceResolution;

	// Backward pass constraints
	std::vector<PlanPoint> backward_planningPoints = _backwardPass(distanceStep);
	ConstraintSequence backward_constraintSequence({});
	for (PlanPoint &point : backward_planningPoints) {
		backward_constraintSequence.addConstraints({
			{point.distance, {fabs(point.motion_dV_dT[0])}}
		});
	}

	// Forward pass
	constraintSequences.push_back(backward_constraintSequence);
	std::vector<PlanPoint> forward_planningPoints = _forwardPass(distanceStep);
	constraintSequences.pop_back();

	// Store points
	profilePoints = forward_planningPoints;

	return *this;
}

double TrajectoryPlanner::getTotalTime() {
	return profilePoints.back().time_seconds;
}

std::pair<double, std::vector<double>> TrajectoryPlanner::getMotionAtTime(double time_seconds) {
	// Empty case
	if ((int) profilePoints.size() < 2) {
		return {0, {0, 0}};
	}

	// Sanitize time
	time_seconds = aespa_lib::genutil::clamp(time_seconds, 0, getTotalTime());

	// Binary search for planning point
	int bs_l, bs_r, bs_m;
	int bs_result = -1;
	bs_l = 0;
	bs_r = (int) profilePoints.size() - 2;
	while (bs_l <= bs_r) {
		bs_m = bs_l + (bs_r - bs_l) / 2;
		PlanPoint node = profilePoints[bs_m];
		if (node.time_seconds <= time_seconds) {
			bs_result = bs_m;
			bs_l = bs_m + 1;
		} else {
			bs_r = bs_m - 1;
		}
	}
	if (bs_result == -1) bs_result = 0;

	// Get segment
	PlanPoint point1 = profilePoints[bs_result];
	PlanPoint point2 = profilePoints[bs_result + 1];

	// Lerp motion
	int motionDegree = fmin(
		(int) point1.motion_dV_dT.size(), (int) point2.motion_dV_dT.size()
	);
	PlanPoint result(-1, 0, std::vector<double>(motionDegree, 0));
	result.distance = aespa_lib::genutil::rangeMap(
		time_seconds,
		point1.time_seconds, point2.time_seconds,
		point1.distance, point2.distance
	);
	for (int i = 0; i < motionDegree; i++) {
		result.motion_dV_dT[i] = aespa_lib::genutil::rangeMap(
			time_seconds,
			point1.time_seconds, point2.time_seconds,
			point1.motion_dV_dT[i], point2.motion_dV_dT[i]
		);
	}

	// Multiply sign
	if (distance_sign == -1) {
		result.distance *= -1;
		for (double &value : result.motion_dV_dT) {
			value *= -1;
		}
	}

	// Return
	return { result.distance, result.motion_dV_dT };
}

}
}
}
