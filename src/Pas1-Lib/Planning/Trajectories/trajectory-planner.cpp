#include "Pas1-Lib/Planning/Trajectories/trajectory-planner.h"

#include "Aespa-Lib/Winter-Utilities/general.h"
#include <stdio.h>
#include <cmath>


namespace pas1_lib {
namespace planning {
namespace trajectories {


// ---------- Plan Point ----------

PlanPoint::PlanPoint(double time_seconds, double distance, std::vector<double> motion_dV_dT)
	: time_seconds(time_seconds), distance(distance),
	motion_dV_dT(motion_dV_dT) {}

PlanPoint &PlanPoint::constrain(Constraint constraint) {
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
	return *this;
}

PlanPoint &PlanPoint::maximizeLastDegree(Constraint constraint) {
	// Expand degree
	int constraintSize = (int) constraint.maxMotion_dV_dT.size();
	if ((int) motion_dV_dT.size() < constraintSize) {
		motion_dV_dT.resize(constraintSize);
	}

	// Maximize last degree
	int lastConstrainedIndex = constraintSize - 1;
	double maxLastMotion_value = constraint.maxMotion_dV_dT[lastConstrainedIndex];
	motion_dV_dT[lastConstrainedIndex] = maxLastMotion_value;

	return *this;
}

double getTimeStepFromDistanceStep(PlanPoint node, double distanceStep) {

	int stepSign = aespa_lib::genutil::signum(distanceStep);

	// Newton's method
	double x = 0;
	double lastX = 0;
	bool isLastOut = false;
	std::pair<double, double> newtonRange = { 0, 1e5 };
	for (int iter = 0; iter < 20; iter++) {
		// Get f(x) and f'(x)
		std::pair<double, std::vector<double>> integ = (
			aespa_lib::genutil::integratePolynomial(node.motion_dV_dT, x)
		);
		double f_x = integ.first - distanceStep;
		double f_p_x = integ.second[0];

		// Extreme conditions
		if (fabs(f_p_x) < 1e-6) {
			// printf("skip %.3f\n", x);
			x += 0.1 * stepSign;
			continue;
		}

		// Not converging
		double dx = -f_x / f_p_x;
		double newX = x + dx;
		bool newXIsOut = newX < newtonRange.first || newtonRange.second < newX;
		lastX = newX;
		if (newXIsOut) {
			// Further out
			int outSign = aespa_lib::genutil::signum(newX - newtonRange.second);
			int deltaOutSign = aespa_lib::genutil::signum(newX - lastX);
			if (isLastOut && outSign == deltaOutSign) {
				break;
			}
			// printf("out %.3f, %.3f %.3f\n", x, newtonRange.first, newtonRange.second);
			x -= 0.1 * outSign;
			isLastOut = true;
			continue;
		}
		isLastOut = false;

		// Converged
		if (fabs(dx) < 1e-5) return x;

		// Iterate
		if (dx < 0) newtonRange.second = fmin(newtonRange.second, x);
		else newtonRange.first = fmax(newtonRange.first, x);
		x = newX;
	}

	// Newton's method on derivative
	// printf("Second\n");
	// printf("%.3f ", distanceStep);
	// for (double a : node.motion_dV_dT) printf("%.3f ", a);
	// printf("\n");
	// printf("CNT: %.3f\n", x);
	x = 0;
	newtonRange = { 0, 1e5 };
	for (int iter = 0; iter < 20; iter++) {
		// Get f(x) and f'(x)
		std::pair<double, std::vector<double>> integ = (
			aespa_lib::genutil::integratePolynomial(node.motion_dV_dT, x)
		);
		double f_x = integ.second[0];
		double f_p_x = integ.second[1];

		// Extreme conditions
		if (fabs(f_p_x) < 1e-6) {
			x += 0.1 * stepSign;
			continue;
		}

		// Not converging
		double dx = -f_x / f_p_x;
		double newX = x + dx;
		bool newXIsOut = newX < newtonRange.first || newtonRange.second < newX;
		lastX = newX;
		if (newXIsOut) {
			// Further out
			int outSign = aespa_lib::genutil::signum(newX - newtonRange.second);
			int deltaOutSign = aespa_lib::genutil::signum(newX - lastX);
			if (isLastOut && outSign == deltaOutSign) {
				break;
			}
			x -= 0.1 * outSign;
			isLastOut = true;
			continue;
		}
		isLastOut = false;

		// Converged
		if (fabs(dx) < 1e-5) return x;

		// Iterate
		if (dx < 0) newtonRange.second = fmin(newtonRange.second, x);
		else newtonRange.first = fmax(newtonRange.first, x);
		x = newX;
	}

	// printf("Newton's method failed :(\n");
	return -1;
}


// ---------- Trajectory Planner ----------

TrajectoryPlanner::TrajectoryPlanner(
	double distance_inches,
	double trackWidth_inches,
	int distanceResolution,
	std::vector<double> startMotion_dInches_dSec,
	std::vector<double> endMotion_dInches_dSec
)
	: trackWidth(trackWidth_inches),
	distanceResolution(distanceResolution),
	startMotion(startMotion_dInches_dSec),
	endMotion(endMotion_dInches_dSec) {
	distance = fabs(distance_inches);
	distance_sign = aespa_lib::genutil::signum(distance_inches);

	curvatureSequence = CurvatureSequence();
};

TrajectoryPlanner::TrajectoryPlanner(
	double distance_inches, double trackWidth_inches, int distanceResolution
)
	: TrajectoryPlanner(distance_inches, trackWidth_inches, distanceResolution, { 0 }, { 0 }) {}

TrajectoryPlanner::TrajectoryPlanner(double distance_inches, double trackWidth_inches)
	: TrajectoryPlanner(distance_inches, trackWidth_inches, 32) {}

TrajectoryPlanner::TrajectoryPlanner(double distance_inches)
	: TrajectoryPlanner(distance_inches, 0) {}

TrajectoryPlanner::TrajectoryPlanner()
	: TrajectoryPlanner(0) {};

TrajectoryPlanner &TrajectoryPlanner::setCurvatureFunction(std::function<double(double)> distanceToCurvature_function) {
	curvatureSequence.points.clear();
	for (int resolutionI = 0; resolutionI <= distanceResolution; resolutionI++) {
		// Get distance
		double x = aespa_lib::genutil::rangeMap(resolutionI, 0, distanceResolution, 0, distance);

		// Store curvature
		double k = distanceToCurvature_function(x);
		curvatureSequence.addPoint(x, k);
	}

	return *this;
}

TrajectoryPlanner &TrajectoryPlanner::smoothenCurvature(double alpha) {
	curvatureSequence.smoothen(alpha);
	return *this;
}

double TrajectoryPlanner::getCurvatureAtDistance(double distance) {
	return curvatureSequence.getCurvatureAtDistance(distance);
}

TrajectoryPlanner &TrajectoryPlanner::addConstraintSequence(ConstraintSequence constraints) {
	constraintSequences.push_back(constraints);
	return *this;
}

TrajectoryPlanner &TrajectoryPlanner::addConstraint_maxMotion(std::vector<double> maxMotion_dV_dT) {
	addConstraintSequence(
		ConstraintSequence()
		.addConstraints({ {0, maxMotion_dV_dT} })
	);
	return *this;
}

TrajectoryPlanner &TrajectoryPlanner::addConstraint_maxAngularMotion(std::vector<double> maxAngularMotion) {
	ConstraintSequence constraintSequence;
	for (int resolutionI = 0; resolutionI <= distanceResolution; resolutionI++) {
		// Get distance
		double x = aespa_lib::genutil::rangeMap(resolutionI, 0, distanceResolution, 0, distance);

		// Get curvature
		double k = getCurvatureAtDistance(x);

		// Max linear velocity = max angular velocity / curvature
		std::vector<double> maxMotion = maxAngularMotion;
		if (fabs(k) < 1e-5) {
			for (auto &value : maxMotion) {
				value = 1e8;
			}
		} else {
			for (auto &value : maxMotion) {
				value /= fabs(k);
			}
		}

		// Add constraint
		constraintSequence.addConstraints({
			{x, maxAngularMotion}
		});
	}

	// Add sequence
	addConstraintSequence(constraintSequence);

	return *this;
}

PlanPoint TrajectoryPlanner::_getNextPlanPoint(PlanPoint node, double distanceStep) {
	// Get minimum constraints
	double oldDistance = node.distance;
	std::vector<Constraint> constraints0 = getConstraintsAtDistance(
		constraintSequences, oldDistance
	);
	std::vector<Constraint> center_constraints0 = getConstraintsAtDistance(
		center_constraintSequences, oldDistance
	);
	std::vector<Constraint> track_constraints0 = getConstraintsAtDistance(
		track_constraintSequences, oldDistance
	);
	Constraint minConstraint0 = getMinimumConstraint(constraints0);
	Constraint center_minConstraint0 = getMinimumConstraint(center_constraints0);
	Constraint track_minConstraint0 = getMinimumConstraint(track_constraints0);


	// Minimum constrain
	node.constrain(minConstraint0);
	node.constrain(center_minConstraint0);

	// Maximize last constrained degree
	node.maximizeLastDegree(minConstraint0);

	// printf("OLD: %.3f ", oldDistance);
	// for (double a : node.motion_dV_dT) {
	// 	printf("%.3f ", a);
	// }
	// printf("\n");

	// Binary search for time step
	double timeStep_seconds = getTimeStepFromDistanceStep(node, fabs(distanceStep));
	if (timeStep_seconds <= 1e-5) {
		printf("SKIPPED\n");
		node.motion_dV_dT = std::vector<double>((int) node.motion_dV_dT.size());
		return node;
	}
	// printf("ST: %.3f, TS: %.3f\n", distanceStep, timeStep_seconds);

	// Integrate
	auto integral = aespa_lib::genutil::integratePolynomial(
		node.motion_dV_dT, timeStep_seconds
	);
	distanceStep = integral.first * aespa_lib::genutil::signum(distanceStep);
	double newDistance = oldDistance + distanceStep;
	double time_seconds = node.time_seconds + timeStep_seconds;
	PlanPoint newNode(time_seconds, newDistance, integral.second);


	// Get minimuum constraints
	std::vector<Constraint> constraints1 = getConstraintsAtDistance(
		constraintSequences, newDistance
	);
	std::vector<Constraint> center_constraints1 = getConstraintsAtDistance(
		center_constraintSequences, newDistance
	);
	std::vector<Constraint> track_constraints1 = getConstraintsAtDistance(
		track_constraintSequences, newDistance
	);

	// Minimum constrain
	Constraint minConstraint1 = getMinimumConstraint(constraints1);
	Constraint center_minConstraint1 = getMinimumConstraint(center_constraints1);
	Constraint track_minConstraint1 = getMinimumConstraint(track_constraints1);
	newNode.constrain(minConstraint1);
	newNode.constrain(center_minConstraint1);

	// Derivative constrain
	for (int i = 1; i < (int) newNode.motion_dV_dT.size(); i++) {
		double deriv = (newNode.motion_dV_dT[i - 1] - node.motion_dV_dT[i - 1]) / timeStep_seconds;
		// newNode.motion_dV_dT[i] = fmin(newNode.motion_dV_dT[i], deriv);
		newNode.motion_dV_dT[i] = deriv;
	}

	// Constrain left & right
	if (trackWidth > 0 && !curvatureSequence.points.empty()) {
		// Get curvature & linear + angular factor
		double curvature = getCurvatureAtDistance(newNode.distance);
		double factor = (1 + fabs(curvature) * trackWidth / 2);

		// Constrain
		newNode.motion_dV_dT = aespa_lib::genutil::multiplyVector(newNode.motion_dV_dT, factor);
		newNode.constrain(minConstraint1);
		newNode.constrain(track_minConstraint1);
		newNode.motion_dV_dT = aespa_lib::genutil::multiplyVector(newNode.motion_dV_dT, 1 / factor);
	}

	// Derivative constrain
	// for (int i = 1; i < (int) newNode.motion_dV_dT.size(); i++) {
	// 	double deriv = (newNode.motion_dV_dT[i - 1] - node.motion_dV_dT[i - 1]) / timeStep_seconds;
	// 	// newNode.motion_dV_dT[i] = fmin(newNode.motion_dV_dT[i], deriv);
	// 	newNode.motion_dV_dT[i] = deriv;
	// }
	newNode.constrain(minConstraint1);

	// printf("NEW: %.3f ", newDistance);
	// for (double a : newNode.motion_dV_dT) {
	// 	printf("%.3f ", a);
	// }
	// printf("\n");

	// TODO: remove sharp deceleration

	// Return
	return newNode;
}

std::vector<PlanPoint> TrajectoryPlanner::_forwardPass(double distanceStep) {
	Constraint minConstraint = getMinimumConstraint(
		getConstraintsAtDistance(constraintSequences, 0)
	);
	std::vector<PlanPoint> planningPoints;
	planningPoints.push_back(
		PlanPoint(0, 0, startMotion)
		.constrain(minConstraint)
		.maximizeLastDegree(minConstraint)
	);

	while (true) {
		// Get info
		PlanPoint lastNode = planningPoints.back();

		// Get planning point
		PlanPoint newNode = _getNextPlanPoint(lastNode, distanceStep);

		// Validate total distance not exceeded
		if (newNode.distance >= distance - distanceStep * 0.5) {
			break;
		}

		// Push planning point
		planningPoints.push_back(newNode);
	}
	PlanPoint finalNode = _getNextPlanPoint(planningPoints.back(), distance - planningPoints.back().distance);
	planningPoints.push_back(finalNode);

	return planningPoints;
}

std::vector<PlanPoint> TrajectoryPlanner::_backwardPass(double distanceStep) {
	Constraint minConstraint = getMinimumConstraint(
		getConstraintsAtDistance(constraintSequences, distance)
	);
	std::vector<PlanPoint> planningPoints;
	planningPoints.push_back(
		PlanPoint(0, distance, endMotion)
		.constrain(minConstraint)
		.maximizeLastDegree(minConstraint)
	);

	while (true) {
		// Get info
		PlanPoint lastNode = planningPoints.back();

		// Get planning point
		PlanPoint newNode = _getNextPlanPoint(lastNode, -distanceStep);

		// Validate total distance not exceeded
		if (newNode.distance <= distanceStep * 0.5) {
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

std::pair<ConstraintSequence, ConstraintSequence> TrajectoryPlanner::constraintSequencesFromPlanPoints(
	std::vector<PlanPoint> nodes
) {
	ConstraintSequence constraintSequence({}, true);
	ConstraintSequence track_constraintSequence({}, true);
	for (PlanPoint &point : nodes) {
		// Center
		constraintSequence.addConstraints({
			{point.distance, {fabs(point.motion_dV_dT[0])}}
		});

		// Track
		if (trackWidth > 0 && !curvatureSequence.points.empty()) {
			double curvature = getCurvatureAtDistance(point.distance);
			double factor = (1 + fabs(curvature) * trackWidth / 2);
			track_constraintSequence.addConstraints({
				{
					point.distance,
					{fabs(point.motion_dV_dT[0]) * factor}
				}
			});
		}
	}
	return { constraintSequence, track_constraintSequence };
}

TrajectoryPlanner &TrajectoryPlanner::calculateMotionProfile() {
	double distanceStep = distance / (double) distanceResolution;

	// Backward pass 1
	std::vector<PlanPoint> backward_planningPoints = _backwardPass(distanceStep);
	auto pass_constraintSequences = constraintSequencesFromPlanPoints(backward_planningPoints);
	center_constraintSequences.push_back(pass_constraintSequences.first);
	track_constraintSequences.push_back(pass_constraintSequences.second);
	if (false) {
		double maxTime = backward_planningPoints[0].time_seconds;
		for (PlanPoint &point : backward_planningPoints) {
			// Fix time
			point.time_seconds = maxTime - point.time_seconds;
		}
		profilePoints = backward_planningPoints;
		return *this;
	}

	std::vector<PlanPoint> forward_planningPoints;
	int extraPasses = 0;
	for (int i = 0; i < extraPasses; i++) {
		// Forward pass
		forward_planningPoints = _forwardPass(distanceStep);
		pass_constraintSequences = constraintSequencesFromPlanPoints(forward_planningPoints);
		center_constraintSequences.push_back(pass_constraintSequences.first);
		track_constraintSequences.push_back(pass_constraintSequences.second);

		// Backward pass
		backward_planningPoints = _backwardPass(distanceStep);
		pass_constraintSequences = constraintSequencesFromPlanPoints(backward_planningPoints);
		center_constraintSequences.push_back(pass_constraintSequences.first);
		track_constraintSequences.push_back(pass_constraintSequences.second);
	}

	// Forward pass 2
	forward_planningPoints = _forwardPass(distanceStep);

	for (int i = 0; i < 1 + extraPasses * 2; i++) {
		center_constraintSequences.pop_back();
		track_constraintSequences.pop_back();
	}

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
		return { 0, {0, 0} };
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
