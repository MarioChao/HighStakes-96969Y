/*
List of problems
- for maximizeNthDegree(), sign = 1 is there to prevent a bug with Newton's method
*/

#include "Pas1-Lib/Planning/Trajectories/trajectory-planner.h"

#include "Aespa-Lib/Winter-Utilities/general.h"
#include <stdio.h>
#include <cmath>


namespace {
using aespa_lib::datas::Linegular;
}


namespace pas1_lib {
namespace planning {
namespace trajectories {


// ---------- Plan Point ----------

PlanPoint::PlanPoint(double time_seconds, double distance, std::vector<double> motion_dV_dT)
	: time_seconds(time_seconds), distance(distance),
	motion_dV_dT(motion_dV_dT) {}

PlanPoint &PlanPoint::constrain(DistanceConstraint constraint) {
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

PlanPoint &PlanPoint::maximizeLastDegree(DistanceConstraint constraint) {
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

PlanPoint &PlanPoint::maximizeNthDegree(
	DistanceConstraint constraint, int dV_dT_degree,
	DistanceConstraint target_rawConstraint, bool maximizeLowerDegrees
) {
	// Validate constraint degree
	if ((int) constraint.maxMotion_dV_dT.size() < dV_dT_degree + 1) {
		maximizeLastDegree(constraint);
		return *this;
	}

	// Expand degree
	if ((int) motion_dV_dT.size() < dV_dT_degree + 1) {
		motion_dV_dT.resize(dV_dT_degree + 1);
	}

	// Maximize degrees
	double endDegree = dV_dT_degree;
	if (maximizeLowerDegrees) endDegree = 0;
	for (int degree = dV_dT_degree; degree >= endDegree; degree--) {
		double maxMotion_value = constraint.maxMotion_dV_dT[degree];

		// Sign
		int sign = degree > 0 ?
			(aespa_lib::genutil::signum(
				target_rawConstraint.maxMotion_dV_dT[degree - 1]
			)
		) : 0;
		// printf("Sign: %d, %.3f\n", sign, target_rawConstraint.maxMotion_dV_dT[degree - 1]);
		sign = 1;
		motion_dV_dT[degree] = maxMotion_value * sign;
	}

	return *this;
}

double getTimeStepFromDistanceStep(PlanPoint node, double distanceStep) {

	int stepSign = aespa_lib::genutil::signum(distanceStep);

	// Newton's method
	auto newtonResult = aespa_lib::genutil::newtonsMethod(
		[&](double x) -> std::pair<double, double> {
		std::pair<double, std::vector<double>> integ = (
			aespa_lib::genutil::integratePolynomial(node.motion_dV_dT, x)
		);
		return { integ.first - distanceStep, integ.second[0] };
	},
		0,
		0.1, stepSign
	);

	if (newtonResult.first) return newtonResult.second;

	// Newton's method on velocity
	newtonResult = aespa_lib::genutil::newtonsMethod(
		[&](double x) -> std::pair<double, double> {
		std::pair<double, std::vector<double>> integ = (
			aespa_lib::genutil::integratePolynomial(node.motion_dV_dT, x)
		);
		return { integ.second[0], integ.second[1] };
	},
		0,
		0.1, stepSign
	);

	if (newtonResult.first) return newtonResult.second;

	// printf("Newton's method failed :(\n");
	return -1;
}

ConstraintSequence planPoints_to_rawConstraintSequence(std::vector<PlanPoint> nodes) {
	ConstraintSequence constraintSequence({}, true);
	for (PlanPoint &point : nodes) {
		constraintSequence.addConstraints({
			{point.distance, point.motion_dV_dT}
		});
	}
	return constraintSequence;
}


// ---------- Trajectory Planner ----------

TrajectoryPlanner::TrajectoryPlanner(
	double distance_inches,
	double trackWidth_inches,
	double planPoint_distanceStep,
	std::vector<double> startMotion_dInches_dSec,
	std::vector<double> endMotion_dInches_dSec
)
	: trackWidth(trackWidth_inches),
	planPoint_distanceStep(planPoint_distanceStep),
	startMotion(startMotion_dInches_dSec),
	endMotion(endMotion_dInches_dSec) {
	totalDistance = std::fabs(distance_inches);
	distance_sign = aespa_lib::genutil::signum(distance_inches);

	curvatureSequence = CurvatureSequence();
	curveSampler = nullptr;
};

TrajectoryPlanner::TrajectoryPlanner(
	double distance_inches, double trackWidth_inches, double planPoint_distanceStep
)
	: TrajectoryPlanner(distance_inches, trackWidth_inches, planPoint_distanceStep, { 0 }, { 0 }) {}

TrajectoryPlanner::TrajectoryPlanner(double distance_inches, double trackWidth_inches)
	: TrajectoryPlanner(distance_inches, trackWidth_inches, 0.1) {}

TrajectoryPlanner::TrajectoryPlanner(double distance_inches)
	: TrajectoryPlanner(distance_inches, 0) {}

TrajectoryPlanner::TrajectoryPlanner()
	: TrajectoryPlanner(0) {};

TrajectoryPlanner &TrajectoryPlanner::setCurvatureFunction(
	std::function<double(double)> distanceToCurvature_function,
	std::vector<double> specificDistances
) {
	// Init & configs
	curvatureSequence.points.clear();
	double distanceStep = planPoint_distanceStep;
	double deltaX = distanceStep;
	int stepSplit = 5;

	// Sort specific distances
	for (double x : specificDistances) {
		double deltaX = 1e-3;

		// Calculate segment curvature
		double avgK = 0;
		double maxK = 0;
		double segmentK = 0;
		for (int stepI = 0; stepI <= stepSplit; stepI++) {
			double dStep = aespa_lib::genutil::rangeMap(
				stepI, 0, stepSplit, 0, deltaX
			) - (deltaX / 2);
			double k = distanceToCurvature_function(x + dStep);
			avgK += k;
			if (std::fabs(k) > std::fabs(maxK)) maxK = k;
		}
		avgK /= (stepSplit + 1);

		// Store curvature
		segmentK = maxK;
		curvatureSequence.addPoint(x, segmentK);
	}

	// Store function
	double x = 0;
	bool lastIteration = false;
	while (true) {
		if (x >= totalDistance) {
			x = totalDistance;
			lastIteration = true;
		}

		// Estimate segment curvature
		double avgK = 0;
		double maxK = 0;
		double segmentK = 0;

		// Shrink segment size
		if (true) {
			for (int stepI = 0; stepI <= stepSplit; stepI++) {
				double dStep = aespa_lib::genutil::rangeMap(
					stepI, 0, stepSplit, 0, deltaX
				) - (deltaX / 2);
				double k = distanceToCurvature_function(x + dStep);
				avgK += k;
				if (std::fabs(k) > std::fabs(maxK)) maxK = k;
			}
			avgK /= (stepSplit + 1);

			// Modify segment size
			// segmentK = avgK;
			segmentK = maxK;
			deltaX = distanceStep / (1 + std::fabs(segmentK));
			deltaX = aespa_lib::genutil::clamp(deltaX, distanceStep / 7.0, distanceStep);
		}

		// Calculate segment curvature
		avgK = 0;
		maxK = 0;
		for (int stepI = 0; stepI <= stepSplit; stepI++) {
			double dStep = aespa_lib::genutil::rangeMap(
				stepI, 0, stepSplit, 0, deltaX
			) - (deltaX / 2);
			double k = distanceToCurvature_function(x + dStep);
			avgK += k;
			if (std::fabs(k) > std::fabs(maxK)) maxK = k;
		}
		avgK /= (stepSplit + 1);

		// Store curvature
		segmentK = maxK;
		curvatureSequence.addPoint(x, segmentK);

		// Increment distance
		x += deltaX;

		if (lastIteration) break;
	}

	return *this;
}

TrajectoryPlanner &TrajectoryPlanner::maxSmoothCurvature(double epsilon) {
	curvatureSequence.maxSmooth(epsilon);
	return *this;
}

double TrajectoryPlanner::getCurvatureAtDistance(double distance) {
	return curvatureSequence.getCurvatureAtDistance(distance);
}

TrajectoryPlanner &TrajectoryPlanner::setSpline(splines::CurveSampler *curveSampler) {
	this->curveSampler = curveSampler;
	return *this;
}

Linegular TrajectoryPlanner::getLinegularAtDistance(double distance) {
	if (curveSampler != nullptr) return curveSampler->getLinegularAtDistance(distance);
	else return Linegular(0, 0, 0);
}

TrajectoryPlanner &TrajectoryPlanner::addCenterConstraintSequence(ConstraintSequence constraints) {
	center_constraintSequences.push_back(constraints);
	return *this;
}

TrajectoryPlanner &TrajectoryPlanner::addTrackConstraintSequence(ConstraintSequence constraints) {
	track_constraintSequences.push_back(constraints);
	return *this;
}

TrajectoryPlanner &TrajectoryPlanner::addCenterConstraints(std::vector<std::pair<double, std::vector<double>>> constraints) {
	addCenterConstraintSequence(
		ConstraintSequence()
		.addConstraints(constraints)
	);
	return *this;
}

TrajectoryPlanner &TrajectoryPlanner::addTrackConstraints(std::vector<std::pair<double, std::vector<double>>> constraints) {
	addTrackConstraintSequence(
		ConstraintSequence()
		.addConstraints(constraints)
	);
	return *this;
}

TrajectoryPlanner &TrajectoryPlanner::addCenterConstraint_maxMotion(std::vector<double> maxMotion_dV_dT) {
	addCenterConstraints({ {0, maxMotion_dV_dT} });
	return *this;
}

TrajectoryPlanner &TrajectoryPlanner::addTrackConstraint_maxMotion(std::vector<double> maxMotion_dV_dT) {
	addTrackConstraints({ {0, maxMotion_dV_dT} });
	return *this;
}

TrajectoryPlanner &TrajectoryPlanner::addCenterConstraint_maxCentripetalAcceleration(double maxCentripetalAceleration) {
	center_trajectoryConstraints.push_back(std::shared_ptr<TrajectoryConstraint>(
		new CentripetalAccelerationConstraint(maxCentripetalAceleration)
	));
	return *this;
}

PlanPoint TrajectoryPlanner::_getNextPlanPoint(
	PlanPoint originalNode,
	double distanceStep,
	int dV_dT_degree
) {
	// Copy node
	PlanPoint node(originalNode);

	// Get info
	double oldDistance = node.distance;
	double originalVelocity = node.motion_dV_dT[0];
	double oldCurvature = getCurvatureAtDistance(oldDistance);
	Linegular oldPose = getLinegularAtDistance(oldDistance);

	// Get minimum constraints
	std::vector<DistanceConstraint> center_constraints0 = getConstraintsAtDistance(
		center_constraintSequences, oldDistance
	);
	std::vector<DistanceConstraint> track_constraints0 = getConstraintsAtDistance(
		track_constraintSequences, oldDistance
	);
	DistanceConstraint center_minConstraint0 = getMinimumConstraint(center_constraints0);
	DistanceConstraint track_minConstraint0 = getMinimumConstraint(track_constraints0);


	// Minimum constrain
	node.constrain(center_minConstraint0);

	std::vector<DistanceConstraint> center_constraints0_1 = getConstraintsAtDistance(
		center_constraintSequences, oldDistance + distanceStep
	);
	DistanceConstraint center_minConstraint0_1 = getMinimumConstraint(center_constraints0_1);
	node.constrain(center_minConstraint0_1);

	// Trajectory constraint
	double trajectoryVelocity0 = getVelocity_trajectoryConstraints(center_trajectoryConstraints, oldPose, oldCurvature, originalVelocity);
	node.motion_dV_dT[0] = std::min(node.motion_dV_dT[0], trajectoryVelocity0);

	// Get target raw constraint
	DistanceConstraint planRaw_constraint0 = planPoint_rawSequence.getConstraintAtDistance(oldDistance);

	// Maximize next constrained degree
	node.maximizeNthDegree(
		center_minConstraint0, dV_dT_degree + 1,
		planRaw_constraint0
	);

	// Constrain left & right
	if (trackWidth > 0 && !curvatureSequence.points.empty()) {
		// Get curvature & linear + angular factor
		double curvature = oldCurvature;
		double factor = (1 + std::fabs(curvature) * trackWidth / 2);

		// Constrain
		node.motion_dV_dT = aespa_lib::genutil::multiplyVector(node.motion_dV_dT, factor);
		node.constrain(track_minConstraint0);
		node.motion_dV_dT = aespa_lib::genutil::multiplyVector(node.motion_dV_dT, 1 / factor);
	}

	// Binary search for time step
	PlanPoint nodeToIntegrate = node;
	double timeStep_seconds = getTimeStepFromDistanceStep(nodeToIntegrate, std::fabs(distanceStep));
	if (timeStep_seconds <= 1e-5) {
		printf("SKIPPED\n");
		node.distance += distanceStep;
		node.time_seconds += 0.1 * aespa_lib::genutil::signum(distanceStep);
		node.motion_dV_dT = std::vector<double>((int) node.motion_dV_dT.size());
		return node;
	}

	// Integrate
	auto integral = aespa_lib::genutil::integratePolynomial(
		node.motion_dV_dT, timeStep_seconds
	);
	// distanceStep = integral.first * aespa_lib::genutil::signum(distanceStep);
	double newDistance = oldDistance + distanceStep;
	double time_seconds = node.time_seconds + timeStep_seconds;
	PlanPoint newNode(time_seconds, newDistance, integral.second);

	// Get new info
	double newOriginalVelocity = newNode.motion_dV_dT[0];
	double newCurvature = getCurvatureAtDistance(newDistance);
	Linegular newPose = getLinegularAtDistance(newDistance);

	// Get minimum constraints
	std::vector<DistanceConstraint> center_constraints1 = getConstraintsAtDistance(
		center_constraintSequences, newDistance
	);
	std::vector<DistanceConstraint> track_constraints1 = getConstraintsAtDistance(
		track_constraintSequences, newDistance
	);

	// Minimum constrain
	DistanceConstraint center_minConstraint1 = getMinimumConstraint(center_constraints1);
	DistanceConstraint track_minConstraint1 = getMinimumConstraint(track_constraints1);
	newNode.constrain(center_minConstraint1);
	// printf("T dis: %.3f, dis: %.3f, vel: %.3f\n", distance, newNode.distance, newNode.motion_dV_dT[0]);

	// Trajectory constraint
	double trajectoryVelocity1 = getVelocity_trajectoryConstraints(center_trajectoryConstraints, newPose, newCurvature, newOriginalVelocity);
	newNode.motion_dV_dT[0] = std::min(newNode.motion_dV_dT[0], trajectoryVelocity1);

	// Derivative constrain
	for (int i = dV_dT_degree + 1; i < (int) newNode.motion_dV_dT.size(); i++) {
		double deriv = (newNode.motion_dV_dT[i - 1] - originalNode.motion_dV_dT[i - 1]) / timeStep_seconds;
		// newNode.motion_dV_dT[i] = fmin(newNode.motion_dV_dT[i], deriv);
		newNode.motion_dV_dT[i] = deriv;
		// printf("dis: %.3f, ts: %.3f, deriv: %.3f\n", newNode.distance, timeStep_seconds, deriv);
	}

	// Constrain left & right
	if (trackWidth > 0 && !curvatureSequence.points.empty()) {
		// Get curvature & linear + angular factor
		double curvature = newCurvature;
		double factor = (1 + std::fabs(curvature) * trackWidth / 2);

		// Scale to track & constrain
		newNode.motion_dV_dT = aespa_lib::genutil::multiplyVector(newNode.motion_dV_dT, factor);
		newNode.constrain(track_minConstraint1);
		newNode.motion_dV_dT = aespa_lib::genutil::multiplyVector(newNode.motion_dV_dT, 1 / factor);
	}

	// Constrain left & right integral (wheel acceleration)
	if (trackWidth > 0 && !curvatureSequence.points.empty()) {
		// Get curvature & track factors
		double curvature0 = oldCurvature;
		double factor0_left = (1 - curvature0 * trackWidth / 2);
		double factor0_right = (1 + curvature0 * trackWidth / 2);
		double curvature1 = newCurvature;
		double factor1_left = (1 - curvature1 * trackWidth / 2);
		double factor1_right = (1 + curvature1 * trackWidth / 2);

		// Get track motion
		std::vector<double> factor0s = { factor0_left, factor0_right };
		std::vector<double> factor1s = { factor1_left, factor1_right };
		for (int index = 0; index < (int) factor0s.size(); index++) {
			double factor0 = factor0s[index];
			double factor1 = factor1s[index];

			// Equations:
			// vwheel_old = v_old * factor0
			// vwheel_new = v_new * factor1
			// vwheel_new - vwheel_old < maxaccel * dt

			// Scale target raw constraint to track
			DistanceConstraint planRaw_track_constraint0 = planRaw_constraint0;
			planRaw_track_constraint0.maxMotion_dV_dT = aespa_lib::genutil::multiplyVector(
				planRaw_track_constraint0.maxMotion_dV_dT, factor0
			);

			// Constrain
			PlanPoint trackNode(originalNode);
			trackNode.motion_dV_dT = aespa_lib::genutil::multiplyVector(
				originalNode.motion_dV_dT, factor0
			);
			trackNode.maximizeNthDegree(
				track_minConstraint0, dV_dT_degree + 1,
				planRaw_track_constraint0
			);
			if (factor0 < 0) trackNode.motion_dV_dT[dV_dT_degree] *= -1;

			// Integrate track motion
			auto trackIntegral = aespa_lib::genutil::integratePolynomial(
				trackNode.motion_dV_dT, timeStep_seconds
			);

			// Scale track integral to center
			trackIntegral.second = aespa_lib::genutil::multiplyVector(trackIntegral.second, 1 / factor1);

			// Constrain
			newNode.constrain(DistanceConstraint(0, aespa_lib::genutil::getAbsolute(trackIntegral.second)));
		}
	}

	newNode.constrain(center_minConstraint1);

	// Return
	return newNode;
}

std::vector<PlanPoint> TrajectoryPlanner::_forwardPass(int dV_dT_degree) {
	DistanceConstraint minConstraint = getMinimumConstraint(
		getConstraintsAtDistance(center_constraintSequences, 0)
	);
	std::vector<PlanPoint> planningPoints;
	planningPoints.push_back(
		PlanPoint(0, 0, startMotion)
		.constrain(minConstraint)
		.maximizeNthDegree(
			minConstraint, dV_dT_degree + 1,
			planPoint_rawSequence.getConstraintAtDistance(0)
		)
	);

	for (int i = 0; i < (int) planPoint_distances.size() - 1; i++) {
		// Get info
		PlanPoint lastNode = planningPoints.back();
		double deltaX = planPoint_distances[i + 1] - planPoint_distances[i];
		if (std::fabs(deltaX) < 1e-5) continue;

		// Get planning point
		PlanPoint newNode = _getNextPlanPoint(lastNode, deltaX, dV_dT_degree);

		// Push planning point
		planningPoints.push_back(newNode);
	}

	return planningPoints;
}

std::vector<PlanPoint> TrajectoryPlanner::_backwardPass(int dV_dT_degree) {
	DistanceConstraint minConstraint = getMinimumConstraint(
		getConstraintsAtDistance(center_constraintSequences, totalDistance)
	);
	std::vector<PlanPoint> planningPoints;
	planningPoints.push_back(
		PlanPoint(0, totalDistance, endMotion)
		.constrain(minConstraint)
		.maximizeNthDegree(
			minConstraint, dV_dT_degree + 1,
			planPoint_rawSequence.getConstraintAtDistance(totalDistance)
		)
	);

	for (int i = (int) planPoint_distances.size() - 2; i >= 0; i--) {
		// Get info
		PlanPoint lastNode = planningPoints.back();
		double deltaX = planPoint_distances[i + 1] - planPoint_distances[i];
		if (std::fabs(deltaX) < 1e-5) continue;

		// Get planning point
		PlanPoint newNode = _getNextPlanPoint(lastNode, -deltaX, dV_dT_degree);

		// Push planning point
		planningPoints.push_back(newNode);
	}

	// Flip acceleration and above
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
			{point.distance, {std::fabs(point.motion_dV_dT[0])}}
			// {point.distance, aespa_lib::genutil::getAbsolute(point.motion_dV_dT)}
		});

		// Track
		if (trackWidth > 0 && !curvatureSequence.points.empty()) {
			double curvature = getCurvatureAtDistance(point.distance);
			double factor = (1 + std::fabs(curvature) * trackWidth / 2);
			// track_constraintSequence.addConstraints({
			// 	{
			// 		point.distance,
			// 		{std::fabs(point.motion_dV_dT[0]) * factor}
			// 	}
			// });
		}
	}
	return { constraintSequence, track_constraintSequence };
}

TrajectoryPlanner &TrajectoryPlanner::calculateMotionProfile() {
	// Get max degree
	int maxdV_dT_degree = 0;
	for (ConstraintSequence &sequence : center_constraintSequences) {
		maxdV_dT_degree = std::fmax(
			maxdV_dT_degree,
			(int) sequence.constraints.front().maxMotion_dV_dT.size()
		);
	}
	for (ConstraintSequence &sequence : track_constraintSequences) {
		maxdV_dT_degree = std::fmax(
			maxdV_dT_degree,
			(int) sequence.constraints.front().maxMotion_dV_dT.size()
		);
	}

	// Generate plan point distances
	double distanceStep = planPoint_distanceStep;
	planPoint_distances.clear();
	for (double x = 0; x <= totalDistance; x += distanceStep) {
		planPoint_distances.push_back(x);
	}
	planPoint_distances.push_back(totalDistance);
	if (trackWidth > 0 && !curvatureSequence.points.empty()) {
		std::vector<double> newPlanPoint_distances = {};
		int p1 = 0;
		for (CurvaturePoint &point : curvatureSequence.points) {
			while (p1 < (int) planPoint_distances.size() && planPoint_distances[p1] < point.distance) {
				newPlanPoint_distances.push_back(planPoint_distances[p1]);
				p1++;
			}
			if (newPlanPoint_distances.empty() || std::fabs(point.distance - newPlanPoint_distances.back()) > 1e-5) {
				newPlanPoint_distances.push_back(point.distance);
			}
		}
		std::vector<double> tmpPlanPoint_distances = {};
		double lastX = -1;
		for (double &point_distance : newPlanPoint_distances) {
			if (point_distance - lastX <= 1e-5) continue;
			tmpPlanPoint_distances.push_back(point_distance);
			lastX = point_distance;
		}
		planPoint_distances = tmpPlanPoint_distances;
	}
	printf("SZ: %d\n", (int) planPoint_distances.size());

	// Initialize plan raw sequence
	planPoint_rawSequence.constraints.clear();
	planPoint_rawSequence.addConstraints({ {0, {1e9}} });

	// Double pass config
	int desiredPasses = 1;
	bool skipFirst = false;

	// Forward-backward passes
	int passesCount = 0;
	std::vector<PlanPoint> backward_planningPoints;
	std::vector<PlanPoint> forward_planningPoints;
	std::pair<ConstraintSequence, ConstraintSequence> pass_constraintSequences;
	for (int degree = 0; degree < maxdV_dT_degree - 1; degree++) {
		for (int pass = 0; pass < desiredPasses; pass++) {
			if (!skipFirst) {
				// Forward pass
				forward_planningPoints = _forwardPass(degree);

				// Clear pass constraints
				for (int i = 0; i < passesCount; i++) {
					center_constraintSequences.pop_back();
					track_constraintSequences.pop_back();
				}
				passesCount = 0;
				// break;

				if (degree == 0 && false) { break; }

				// Constrain
				passesCount++;
				pass_constraintSequences = constraintSequencesFromPlanPoints(forward_planningPoints);
				center_constraintSequences.push_back(pass_constraintSequences.first);
				track_constraintSequences.push_back(pass_constraintSequences.second);
				planPoint_rawSequence = planPoints_to_rawConstraintSequence(forward_planningPoints);
			}
			skipFirst = false;

			// Backward pass
			backward_planningPoints = _backwardPass(degree);

			// Clear pass constraints
			for (int i = 0; i < passesCount; i++) {
				center_constraintSequences.pop_back();
				track_constraintSequences.pop_back();
			}
			passesCount = 0;

			if (degree == 0 && false) {
				double maxTime = backward_planningPoints[0].time_seconds;
				for (PlanPoint &point : backward_planningPoints) {
					// Fix time
					point.time_seconds = maxTime - point.time_seconds;
				}
				profilePoints = backward_planningPoints;
				return *this;
			}

			// Constrain
			passesCount++;
			pass_constraintSequences = constraintSequencesFromPlanPoints(backward_planningPoints);
			center_constraintSequences.push_back(pass_constraintSequences.first);
			track_constraintSequences.push_back(pass_constraintSequences.second);
			planPoint_rawSequence = planPoints_to_rawConstraintSequence(backward_planningPoints);
		}
	}

	// Forward pass
	forward_planningPoints = _forwardPass(maxdV_dT_degree - 2);

	for (int i = 0; i < passesCount; i++) {
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
	if ((int) profilePoints.size() < 2) return { 0, {0, 0} };

	// Out of range case
	if (time_seconds < 0) return { 0, {0, 0} };
	if (getTotalTime() < time_seconds) return { totalDistance * distance_sign, {0, 0} };

	// Sanitize time
	time_seconds = aespa_lib::genutil::clamp(time_seconds, 0, getTotalTime());

	// Binary search for planning point
	int bs_l, bs_r, bs_m;
	int bs_result = 0;
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
