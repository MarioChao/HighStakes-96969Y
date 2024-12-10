#pragma once

#include <vector>
#include <algorithm>

// Forward declaration

class CurveSampler;


// Namespace

namespace trajectory {
	using merged_kinematics = std::vector<std::pair<double, std::pair<
		std::pair<bool, std::vector<double>>,
		std::pair<bool, std::vector<double>>
	>>>;
}


// Class

class TrajectoryPlanner {
public:
	TrajectoryPlanner(double totalDistance = 0);

	void _onInit(double totalDistance);

	// Add motion constraints according to a curve's curvature.
	TrajectoryPlanner &autoSetMotionConstraints(
		CurveSampler sampler, double minVelocity, double maxVelocity,
		double maxAccel, double maxDecel,
		int resolution = 30,
		double leftRightWheelDistance = -1
	);

	// Add motion constraints. Call in ascending order of `startDistance` please.
	TrajectoryPlanner &addDesiredMotionConstraints(
		double startDistance, double maxVelocity,
		double maxAccel, double maxDecel
	);

	std::vector<std::pair<double, std::vector<double>>> _getForwardKinematics();
	std::vector<std::pair<double, std::vector<double>>> _getBackwardKinematics();
	trajectory::merged_kinematics _getMergedForwardBackward();
	std::vector<std::pair<double, std::vector<double>>> _getCombinedKinematics();
	TrajectoryPlanner &calculateMotion();

	std::vector<double> getMotionAtTime(double time);

	double getTotalTime();

private:
	std::vector<std::pair<double, std::vector<double>>> distance_motionConstraints;

	std::vector<std::pair<double, std::vector<double>>> time_kinematics;
	double totalDistance;
};
