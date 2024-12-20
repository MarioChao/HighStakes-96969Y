#include "GraphUtilities/trajectoryPlanner.h"

#include "GraphUtilities/curveSampler.h"

#include "Utilities/generalUtility.h"
#include "Utilities/robotInfo.h"
#include "Utilities/fieldInfo.h"

#include <stdio.h>

namespace {
	bool debugPrint = false;
}

TrajectoryPlanner::TrajectoryPlanner(double totalDistance) {
	_onInit(totalDistance);
}

void TrajectoryPlanner::_onInit(double totalDistance) {
	distance_motionConstraints.clear();
	this->totalDistance = totalDistance;
}

TrajectoryPlanner &TrajectoryPlanner::autoSetMotionConstraints(
	CurveSampler sampler, double minVelocity, double maxVelocity,
	double maxAccel, double maxDecel,
	int resolution,
	double leftRightWheelDistance
) {
	// Clear motion constraints
	distance_motionConstraints.clear();

	// Preprocess config
	if (leftRightWheelDistance < 0) {
		leftRightWheelDistance = botinfo::robotLengthIn * (1.0 / field::tileLengthIn);
	}

	// Get sampler info
	double pathStart = sampler.getDistanceRange().first;
	double pathEnd = sampler.getDistanceRange().second;
	UniformCubicSpline spline = sampler.getSpline();

	// Set motion constraints for segments
	for (int i = 0; i < resolution; i++) {
		// Get the segment distance
		double segmentDistance_start = genutil::rangeMap(i, 0, resolution, pathStart, pathEnd);

		// Get estimated curvature at distance
		double curvature = 0;
		for (double j = i; j < i + 1; j += 0.1) {
			double subSegment_distance = genutil::rangeMap(j, 0, resolution, pathStart, pathEnd);
			double subSegment_curvature = spline.getCurvatureAt(sampler.distanceToParam(subSegment_distance));
			// Use maximum
			curvature = std::max(curvature, std::fabs(subSegment_curvature));
		}

		// Calculate velocity used for rotation
		// w = v/r = v*k
		// v_wheel = w*r
		double rotationLinearVelocity = maxVelocity * curvature * (leftRightWheelDistance / 2.0);
		rotationLinearVelocity *= 1.2;

		// Calculate constraint values
		// double nonChangingFactor = 0.3;
		// double segmentMaxVelocity = maxVelocity * (nonChangingFactor / (nonChangingFactor + curvature));
		double segmentMaxVelocity = maxVelocity - rotationLinearVelocity;
		segmentMaxVelocity = genutil::clamp(segmentMaxVelocity, minVelocity, maxVelocity);
		// printf("d: %.3f, curva: %.3f, vel: %.3f, ang: %.3f\n", segmentDistance_start, curvature, segmentMaxVelocity, rotationLinearVelocity);

		// Add constraint
		addDesiredMotionConstraints(segmentDistance_start, segmentMaxVelocity, maxAccel, maxDecel);
	}

	// Method chaining
	return *this;
}

TrajectoryPlanner &TrajectoryPlanner::addDesiredMotionConstraints(
	double startDistance, double maxVelocity,
	double maxAccel, double maxDecel
) {
	std::pair<double, std::vector<double>> constraint = {startDistance, {maxVelocity, maxAccel, maxDecel}};
	distance_motionConstraints.push_back(constraint);

	// Method chaining
	return *this;
}

std::vector<std::pair<double, std::vector<double>>> TrajectoryPlanner::_getForwardKinematics() {
	// Initialize
	std::vector<std::pair<double, std::vector<double>>> distance_kinematics = {};

	// Look through each constraint segment
	const int segmentCount = distance_motionConstraints.size();
	double travellingVelocity = 0;
	for (int segment = 0; segment < (int) segmentCount; segment++) {
		// Get segment info for [segment, segment + 1]
		const double distanceStart = distance_motionConstraints[segment].first;
		const double distanceEnd = (segment == segmentCount - 1) ? totalDistance : distance_motionConstraints[segment + 1].first;
		const double segmentDistance = distanceEnd - distanceStart;
		std::vector<double> &motionConstraints = distance_motionConstraints[segment].second;
		const double maxVelocity = motionConstraints[0];
		const double maxAccel = motionConstraints[1];
		// const double maxAccel = motionConstraints[2];

		// Push increasing-velocity kinematics info
		if (travellingVelocity < maxVelocity) {
			distance_kinematics.push_back({distanceStart, {travellingVelocity, maxAccel}});
			// printf("Push: %.3f, %.3f, %.3f\n", distanceStart, travellingVelocity, maxAccel);
		}

		// Update velocity using constraint
		travellingVelocity = std::min(travellingVelocity, maxVelocity);

		// Compute kinematics at constant-velocity
		// Note: the acceleration constraint is constant
		// vf^2 = vi^2 + 2aΔs
		// Δs = (vf^2 - vi^2) / 2a
		const double maxVelDistance = (std::pow(maxVelocity, 2) - std::pow(travellingVelocity, 2)) / (2 * maxAccel);

		// Push constant-velocity kinematics info
		if (maxVelDistance < segmentDistance) {
			distance_kinematics.push_back({distanceStart + maxVelDistance, {maxVelocity, 0}});
			// printf("Push max: %.3f, %.3f, 0\n", distanceStart + maxVelDistance, maxVelocity);
		}

		// Update velocity using acceleration
		// vf^2 = vi^2 + 2aΔs
		// vf = √(vi^2 + 2aΔs)
		const double incVelDistance = std::min(maxVelDistance, segmentDistance);
		travellingVelocity = std::sqrt(std::pow(travellingVelocity, 2) + 2 * maxAccel * incVelDistance);
	}

	// Return result
	return distance_kinematics;
}

std::vector<std::pair<double, std::vector<double>>> TrajectoryPlanner::_getBackwardKinematics() {
	// Initialize
	std::vector<std::pair<double, std::vector<double>>> distance_kinematics = {};

	// Look through each constraint segment
	const int segmentCount = distance_motionConstraints.size();
	double travellingVelocity = 0;
	for (int segment = segmentCount - 1; segment >= 0; segment--) {
		// Get segment info for [segment, segment + 1]
		const double distanceStart = (segment == segmentCount - 1) ? 0 : totalDistance - distance_motionConstraints[segment + 1].first;
		const double distanceEnd = totalDistance - distance_motionConstraints[segment].first;
		const double segmentDistance = distanceEnd - distanceStart;
		std::vector<double> &motionConstraints = distance_motionConstraints[segment].second;
		const double maxVelocity = motionConstraints[0];
		// const double maxAccel = motionConstraints[1];
		const double maxAccel = motionConstraints[2];

		// Push increasing-velocity kinematics info
		if (travellingVelocity < maxVelocity) {
			distance_kinematics.push_back({distanceStart, {travellingVelocity, maxAccel}});
			// printf("Push: %.3f, %.3f, %.3f\n", distanceStart, travellingVelocity, maxAccel);
		}

		// Update velocity using constraint
		travellingVelocity = std::min(travellingVelocity, maxVelocity);

		// Compute kinematics at constant-velocity
		// Note: the acceleration constraint is constant
		// vf^2 = vi^2 + 2aΔs
		// Δs = (vf^2 - vi^2) / 2a
		const double maxVelDistance = (std::pow(maxVelocity, 2) - std::pow(travellingVelocity, 2)) / (2 * maxAccel);

		// Push constant-velocity kinematics info
		if (maxVelDistance < segmentDistance) {
			distance_kinematics.push_back({distanceStart + maxVelDistance, {maxVelocity, 0}});
			// printf("Push max: %.3f, %.3f, 0\n", distanceStart + maxVelDistance, maxVelocity);
		}

		// Update velocity using acceleration
		// vf^2 = vi^2 + 2aΔs
		// vf = √(vi^2 + 2aΔs)
		const double incVelDistance = std::min(maxVelDistance, segmentDistance);
		travellingVelocity = std::sqrt(std::pow(travellingVelocity, 2) + 2 * maxAccel * incVelDistance);
	}

	// Reverse distances
	for (int i = 0; i < (int) distance_kinematics.size(); i++) {
		// Swap segment endpoint & reverse the distance
		double distanceEnd = (i == (int) distance_kinematics.size() - 1) ? totalDistance : distance_kinematics[i + 1].first;
		const double segmentDistance = distanceEnd - distance_kinematics[i].first;
		distance_kinematics[i].first = totalDistance - distanceEnd;

		// Calculate flipped velocity
		const double v = distance_kinematics[i].second[0];
		const double a = distance_kinematics[i].second[1];
		distance_kinematics[i].second[0] = std::sqrt(std::pow(v, 2) + 2 * a * segmentDistance);

		// Negate acceleration
		distance_kinematics[i].second[1] *= -1;
	}
	std::reverse(distance_kinematics.begin(), distance_kinematics.end());

	// Return result
	return distance_kinematics;
}

trajectory::merged_kinematics TrajectoryPlanner::_getMergedForwardBackward() {
	// Forward kinematics
	std::vector<std::pair<double, std::vector<double>>> forward_distance_kinematics = _getForwardKinematics();

	// Backward kinematics
	std::vector<std::pair<double, std::vector<double>>> backward_distance_kinematics = _getBackwardKinematics();

	// Size info
	const int forward_size = (int) forward_distance_kinematics.size();
	const int backward_size = (int) backward_distance_kinematics.size();

	// for (int i = 0; i < forward_size; i++) {
	// 	printf("%.3f, %.3f, %.3f\n", forward_distance_kinematics[i].first, forward_distance_kinematics[i].second[0], forward_distance_kinematics[i].second[1]);
	// }
	// for (int i = 0; i < backward_size; i++) {
	// 	printf("%.3f, %.3f, %.3f\n", backward_distance_kinematics[i].first, backward_distance_kinematics[i].second[0], backward_distance_kinematics[i].second[1]);
	// }

	// Merge forward and backward kinematics
	trajectory::merged_kinematics bothside_distance_kinematics = {};
	int forward_index, backward_index;
	forward_index = backward_index = 0;
	while (forward_index < forward_size && backward_index < backward_size) {
		// Get info
		const double forward_distance = forward_distance_kinematics[forward_index].first;
		const std::vector<double> &forward_kinematics = forward_distance_kinematics[forward_index].second;
		const double backward_distance = backward_distance_kinematics[backward_index].first;
		const std::vector<double> &backward_kinematics = backward_distance_kinematics[backward_index].second;

		// Push smaller distance
		if (genutil::isWithin(forward_distance, backward_distance, 1e-7)) {
			bothside_distance_kinematics.push_back({forward_distance, {{1, forward_kinematics}, {1, backward_kinematics}}});
			forward_index++;
			backward_index++;
		} else if (forward_distance < backward_distance) {
			bothside_distance_kinematics.push_back({forward_distance, {{1, forward_kinematics}, {0, {0, 0}}}});
			forward_index++;
		} else {
			bothside_distance_kinematics.push_back({backward_distance, {{0, {0, 0}}, {1, backward_kinematics}}});
			backward_index++;
		}
	}
	while (forward_index < forward_size) {
		// Get info
		const double forward_distance = forward_distance_kinematics[forward_index].first;
		const std::vector<double> &forward_kinematics = forward_distance_kinematics[forward_index].second;

		// Push
		bothside_distance_kinematics.push_back({forward_distance, {{1, forward_kinematics}, {0, {0, 0}}}});
		forward_index++;
	}
	while (backward_index < backward_size) {
		// Get info
		const double backward_distance = backward_distance_kinematics[backward_index].first;
		const std::vector<double> &backward_kinematics = backward_distance_kinematics[backward_index].second;

		// Push
		bothside_distance_kinematics.push_back({backward_distance, {{0, {0, 0}}, {1, backward_kinematics}}});
		backward_index++;
	}

	// Return result
	return bothside_distance_kinematics;
}

std::vector<std::pair<double, std::vector<double>>> TrajectoryPlanner::_getCombinedKinematics() {
	// Get merged forward and backward kinematics
	trajectory::merged_kinematics merged_distance_kinematics = _getMergedForwardBackward();

	// for (int i = 0; i < (int) merged_distance_kinematics.size(); i++) {
	// 	auto &distance_kinematics = merged_distance_kinematics[i];
	// 	printf("%.3f, %d, %d\n", distance_kinematics.first, distance_kinematics.second.first.first, distance_kinematics.second.second.first);
	// }

	// Initialize forward and backward travelling kinematics
	std::vector<double> forwardTravellingKinematics, backwardTravellingKinematics;
	forwardTravellingKinematics = backwardTravellingKinematics = {0, 0};
	double lastDistance = 0;
	double lastIntersectionDistance = -1;

	// Initialize result
	std::vector<std::pair<double, std::vector<double>>> combined_distance_kinematics = {};

	// Get minimized kinematics
	for (auto &distance_kinematics : merged_distance_kinematics) {
		if (debugPrint) printf("K: %.3f, %d, %d\n", distance_kinematics.first, distance_kinematics.second.first.first, distance_kinematics.second.second.first);

		// Check if intersection occured before distance
		const double distanceStart = distance_kinematics.first;
		if (lastDistance < lastIntersectionDistance && lastIntersectionDistance < distanceStart) {
			// Calculate intersection velocity
			double intersectionAccel = backwardTravellingKinematics[1];
			double intersectionVelocity = std::sqrt(
				std::pow(backwardTravellingKinematics[0], 2)
				+ 2 * intersectionAccel * (lastIntersectionDistance - lastDistance)
			);

			// Push backward kinematics at intersection
			combined_distance_kinematics.push_back({lastIntersectionDistance, {intersectionVelocity, intersectionAccel}});
			if (debugPrint) printf("inxt: %.3f, %.3f\n", lastIntersectionDistance, intersectionVelocity);
		}

		// Calculate travel distance
		const double travelDistance = distanceStart - lastDistance;
		lastDistance = distanceStart;

		// Update travelling kinematics
		const auto &_forwardKinematics = distance_kinematics.second.first;
		const auto &_backwardKinematics = distance_kinematics.second.second;
		if (_forwardKinematics.first) {
			forwardTravellingKinematics = _forwardKinematics.second;
		} else {
			forwardTravellingKinematics[0] = std::sqrt(
				std::pow(forwardTravellingKinematics[0], 2)
				+ 2 * forwardTravellingKinematics[1] * travelDistance
			);
		}
		if (_backwardKinematics.first) {
			backwardTravellingKinematics = _backwardKinematics.second;
		} else {
			backwardTravellingKinematics[0] = std::sqrt(
				std::pow(backwardTravellingKinematics[0], 2)
				+ 2 * backwardTravellingKinematics[1] * travelDistance
			);
		}

		// Aliases for travelling kinematics
		auto &v = forwardTravellingKinematics[0];
		auto &a = forwardTravellingKinematics[1];
		auto &u = backwardTravellingKinematics[0];
		auto &b = backwardTravellingKinematics[1];

		// Push new kinematics info
		if (combined_distance_kinematics.empty()) {
			combined_distance_kinematics.push_back({distanceStart, {forwardTravellingKinematics}});
			if (debugPrint) printf("1st: %.3f, %.3f\n", distanceStart, v);
		} else {
			if (debugPrint) printf("start1: %.3f, %.6f, %.3f\n", distanceStart, v, a);
			if (debugPrint) printf("start2: %.3f, %.6f, %.3f\n", distanceStart, u, b);
			if (genutil::isWithin(v, u, 1e-5)) {
				combined_distance_kinematics.push_back({distanceStart, {backwardTravellingKinematics}});
				if (debugPrint) printf("chose 2\n");
			} else if (v < u) {
				combined_distance_kinematics.push_back({distanceStart, {forwardTravellingKinematics}});
				if (debugPrint) printf("chose 1\n");
			} else {
				combined_distance_kinematics.push_back({distanceStart, {backwardTravellingKinematics}});
				if (debugPrint) printf("chose 2\n");
			}
		}

		// Calculate new intersection (if it exists)
		bool hasNewIntersection = !(forwardTravellingKinematics[1] == 0 && backwardTravellingKinematics[1] == 0);
		if (hasNewIntersection) {
			// w^2 = vi^2 + 2aΔs = ui^2 + 2bΔs
			// vi^2 + 2aΔs = ui^2 + 2bΔs
			// Δs = (ui^2 - vi^2) / (2a - 2b)
			lastIntersectionDistance = distanceStart + (std::pow(u, 2) - std::pow(v, 2)) / (2 * a - 2 * b);
		} else {
			lastIntersectionDistance = -1;
		}
	}

	// Check for final intersection before total distance
	{
		const double distanceStart = totalDistance;
		if (lastDistance < lastIntersectionDistance && lastIntersectionDistance < distanceStart) {
			// Calculate intersection velocity
			double intersectionAccel = backwardTravellingKinematics[1];
			double intersectionVelocity = std::sqrt(
				std::pow(backwardTravellingKinematics[0], 2)
				+ 2 * intersectionAccel * (lastIntersectionDistance - lastDistance)
			);

			// Push backward kinematics at intersection
			combined_distance_kinematics.push_back({lastIntersectionDistance, {intersectionVelocity, intersectionAccel}});
			if (debugPrint) printf("inxt: %.3f, %.3f\n", lastIntersectionDistance, intersectionVelocity);
		}
	}

	// Return result
	return combined_distance_kinematics;
}

TrajectoryPlanner &TrajectoryPlanner::calculateMotion() {
	// Get combined kinematics
	std::vector<std::pair<double, std::vector<double>>> combined_distance_kinematics = _getCombinedKinematics();

	// Initialize result
	time_kinematics = {};
	double cumulativeTime = 0;

	// Convert kinematics based on time
	// t = Δv / a = 2Δd / (vi + vf)
	const int segmentCount = (int) combined_distance_kinematics.size();
	for (int segmentIndex = 0; segmentIndex < segmentCount; segmentIndex++) {
		// Get kinematics info
		auto &distance_kinematics = combined_distance_kinematics[segmentIndex];
		const double d1 = distance_kinematics.first;
		const double d2 = (segmentIndex == segmentCount - 1) ? totalDistance : combined_distance_kinematics[segmentIndex + 1].first; 
		const double v = distance_kinematics.second[0];
		const double u = (segmentIndex == segmentCount - 1) ? 0 : combined_distance_kinematics[segmentIndex + 1].second[0];
		const double a = distance_kinematics.second[1];

		// Push time kinematics
		time_kinematics.push_back({cumulativeTime, {d1, v, a}});
		if (debugPrint) printf("%.3f, %.3f, %.3f, %.3f\n", cumulativeTime, d1, v, a);

		// Get and update time
		if (v + u == 0 && a == 0) {
			printf("Error in trajectory!\n");
			return *this;
		}
		const double time = (a != 0) ? (u - v) / a : 2 * (d2 - d1) / (v + u);
		cumulativeTime += time;
	}

	// Final zero time
	time_kinematics.push_back({cumulativeTime, {totalDistance, 0, 0}});

	// Method chaining
	return *this;
}

std::vector<double> TrajectoryPlanner::getMotionAtTime(double time) {
	// Validate stored motion
	if (time_kinematics.empty()) {
		return {0, 0, 0};
	}

	// Validate completion
	if (time > time_kinematics.back().first) {
		return time_kinematics.back().second;
	}

	// Binary search for the segment that contains the time
	int bL, bR;
	bL = 0;
	bR = (int) time_kinematics.size() - 1;
	int foundL = 0;
	while (bL <= bR) {
		int bM = bL + (bR - bL) / 2;
		double nodeTime = time_kinematics[bM].first;
		if (nodeTime <= time) {
			foundL = bM;
			bL = bM + 1;
		} else {
			bR = bM - 1;
		}
	}

	// Calculate the motion at that time
	double segmentDeltaTime = time - time_kinematics[foundL].first;
	std::vector<double> nodeKinematics = time_kinematics[foundL].second;
	std::vector<double> motion(3);
	motion[2] = nodeKinematics[2];
	motion[1] = nodeKinematics[1] + nodeKinematics[2] * segmentDeltaTime;
	motion[0] = nodeKinematics[0] + nodeKinematics[1] * segmentDeltaTime + 0.5 * nodeKinematics[2] * pow(segmentDeltaTime, 2);

	// Return result
	return motion;
}

double TrajectoryPlanner::getTotalTime() {
	return time_kinematics.back().first;
}
