#pragma once

#include <vector>
#include <algorithm>

class TrajectoryPlanner {
public:
	TrajectoryPlanner(double totalDistance);

	void _onInit(double totalDistance);

	// Add motion constraints. Call in ascending order of `startDistance` please.
	void addDesiredMotionConstraints(
		double startDistance, double maxVelocity,
		double maxAccel, double maxDecel
	);

	std::vector<std::pair<double, std::vector<double>>> _getForwardKinematics();
	std::vector<std::pair<double, std::vector<double>>> _getBackwardKinematics();
	std::vector<std::pair<double, std::pair<std::vector<double>, std::vector<double>>>> _getMergedForwardBackward();
	std::vector<std::pair<double, std::vector<double>>> _getCombinedKinematics();
	void calculateMotion();

	std::vector<double> getMotionAtTime(double time);

private:
	std::vector<std::pair<double, std::vector<double>>> distance_motionConstraints;

	std::vector<std::pair<double, std::vector<double>>> time_kinematics;
	double totalDistance;
};
