#include "GraphUtilities/trajectoryPlanner.h"

TrajectoryPlanner::TrajectoryPlanner(double totalDistance) {
	_onInit(totalDistance);
}

void TrajectoryPlanner::_onInit(double totalDistance) {
	// distance_motionConstraints.clear();
	this->totalDistance = totalDistance;
}

void TrajectoryPlanner::addDesiredMotionConstraints(
	double startDistance, double maxVelocity,
	double maxAccel, double maxDecel
) {
	std::pair<double, std::vector<double>> constraint = std::make_pair(
		totalDistance,
		std::vector<double>({maxVelocity, maxAccel, maxDecel})
	);
	distance_motionConstraints.push_back(constraint);
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
			distance_kinematics.push_back(std::make_pair(
				distanceStart,
				std::vector<double>({travellingVelocity, maxAccel})
			));
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
			distance_kinematics.push_back(std::make_pair(
				distanceStart + maxVelDistance,
				std::vector<double>({maxVelocity, 0})
			));
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
		const double distanceStart = (segment == segmentCount - 1) ? 0 : totalDistance - distance_motionConstraints[segment].first;
		const double distanceEnd = totalDistance - distance_motionConstraints[segment].first;
		const double segmentDistance = distanceEnd - distanceStart;
		std::vector<double> &motionConstraints = distance_motionConstraints[segment].second;
		const double maxVelocity = motionConstraints[0];
		// const double maxAccel = motionConstraints[1];
		const double maxAccel = motionConstraints[2];

		// Push increasing-velocity kinematics info
		if (travellingVelocity < maxVelocity) {
			distance_kinematics.push_back(std::make_pair(
				distanceStart,
				std::vector<double>({travellingVelocity, maxAccel})
			));
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
			distance_kinematics.push_back(std::make_pair(
				distanceStart + maxVelDistance,
				std::vector<double>({maxVelocity, 0})
			));
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
		distance_kinematics[i].first = totalDistance - distanceEnd;

		// Negate acceleration
		distance_kinematics[i].second[1] *= -1;
	}
	std::reverse(distance_kinematics.begin(), distance_kinematics.end());

	// Return result
	return distance_kinematics;
}

std::vector<std::pair<double, std::pair<std::vector<double>, std::vector<double>>>> TrajectoryPlanner::_getMergedForwardBackward() {
	// Forward kinematics
	std::vector<std::pair<double, std::vector<double>>> forward_distance_kinematics = _getForwardKinematics();

	// Backward kinematics
	std::vector<std::pair<double, std::vector<double>>> backward_distance_kinematics = _getBackwardKinematics();

	// Size info
	const int forward_size = (int) forward_distance_kinematics.size();
	const int backward_size = (int) forward_distance_kinematics.size();

	// Merge forward and backward kinematics
	std::vector<std::pair<double, std::pair<std::vector<double>, std::vector<double>>>> bothside_distance_kinematics = {};
	int forward_index, backward_index;
	forward_index = backward_index = 0;
	while (forward_index < forward_size && backward_index <= backward_size) {
		// Get info
		const double forward_distance = forward_distance_kinematics[forward_index].first;
		const std::vector<double> &forward_kinematics = forward_distance_kinematics[forward_index].second;
		const double backward_distance = forward_distance_kinematics[backward_index].first;
		const std::vector<double> &backward_kinematics = backward_distance_kinematics[backward_index].second;

		// Push smaller distance
		if (std::fabs(forward_distance - backward_distance) < 1e7) {
			bothside_distance_kinematics.push_back({forward_distance, {forward_kinematics, backward_kinematics}});
			forward_index++;
			backward_index++;
		} else if (forward_distance < backward_distance) {
			bothside_distance_kinematics.push_back({forward_distance, {forward_kinematics, {0, 0}}});
			forward_index++;
		} else {
			bothside_distance_kinematics.push_back({backward_distance, {{0, 0}, backward_kinematics}});
			backward_index++;
		}
	}
	while (forward_index < forward_size) {
		// Get info
		const double forward_distance = forward_distance_kinematics[forward_index].first;
		const std::vector<double> &forward_kinematics = forward_distance_kinematics[forward_index].second;

		// Push
		bothside_distance_kinematics.push_back({forward_distance, {forward_kinematics, {0, 0}}});
		forward_index++;
	}
	while (backward_index < backward_size) {
		// Get info
		const double backward_distance = forward_distance_kinematics[backward_index].first;
		const std::vector<double> &backward_kinematics = backward_distance_kinematics[backward_index].second;

		// Push
		bothside_distance_kinematics.push_back({backward_distance, {{0, 0}, backward_kinematics}});
		backward_index++;
	}

	// Return result
	return bothside_distance_kinematics;
}

std::vector<std::pair<double, std::vector<double>>> TrajectoryPlanner::_getCombinedKinematics() {
	// Get merged forward and backward
	std::vector<std::pair<double, std::pair<std::vector<double>, std::vector<double>>>> merged_distance_kinematics = _getMergedForwardBackward();

	// TODO: Get minimized kinematics
	std::vector<std::pair<double, std::vector<double>>> combined_distance_kinematics = {};
	std::vector<double> forwardTravellingMotion, backwardTravellingMotion;
	forwardTravellingMotion = backwardTravellingMotion = {0, 0, 0};

	// Return result
	return combined_distance_kinematics;
}

void TrajectoryPlanner::calculateMotion() {
	// Get combined kinematics
	std::vector<std::pair<double, std::vector<double>>> combined_distance_kinematics = {};

	// Convert to kinematics based on time
	time_kinematics = {};
}

std::vector<double> TrajectoryPlanner::getMotionAtTime(double time) {
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
			bM = bL + 1;
		} else {
			bM = bR - 1;
		}
	}

	// Calculate the motion at that time
	double segmentDeltaTime = time - time_kinematics[foundL].first;
	std::vector<double> nodeKinematics = time_kinematics[foundL].second;
	std::vector<double> motion(3);
	motion[2] = nodeKinematics[2];
	motion[1] = nodeKinematics[1] + motion[2] * segmentDeltaTime;
	motion[0] = nodeKinematics[0] + motion[1] * segmentDeltaTime + 0.5 * motion[2] * pow(segmentDeltaTime, 2);

	// Return result
	return motion;
}
