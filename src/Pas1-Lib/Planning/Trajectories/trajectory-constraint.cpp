#include "Pas1-Lib/Planning/Trajectories/trajectory-constraint.h"

#include <cmath>


namespace {
using aespa_lib::datas::Linegular;
using aespa_lib::geometry::Polygon2D;
}


namespace pas1_lib {
namespace planning {
namespace trajectories {


// ---------- Trajectory Constraint ----------

double TrajectoryConstraint::calculateMaxVelocity(aespa_lib::datas::Linegular pose, double curvature, double velocity) {
	return velocity;
}

double getVelocity_trajectoryConstraints(std::vector<std::shared_ptr<TrajectoryConstraint>> constraints, aespa_lib::datas::Linegular pose, double curvature, double velocity) {
	double result = velocity;
	for (std::shared_ptr<TrajectoryConstraint> &constraint : constraints) {
		result = std::min(result, constraint.get()->calculateMaxVelocity(pose, curvature, velocity));
	}
	return result;
}


CentripetalAccelerationConstraint::CentripetalAccelerationConstraint(double maxCentripetalAcceleration)
	: maxCentripetalAcceleration(maxCentripetalAcceleration) {}

double CentripetalAccelerationConstraint::calculateMaxVelocity(aespa_lib::datas::Linegular pose, double curvature, double velocity) {
	if (std::fabs(curvature) < 1e-6) return velocity;
	return std::sqrt(maxCentripetalAcceleration / std::fabs(curvature));
}


PolygonRegionConstraint::PolygonRegionConstraint(Polygon2D polygon, double maxVelocity)
	: polygon(polygon), maxVelocity(maxVelocity) {}

double PolygonRegionConstraint::calculateMaxVelocity(aespa_lib::datas::Linegular pose, double curvature, double velocity) {
	if (polygon.containsPoint(pose.getPosition())) return maxVelocity;
	return velocity;
}


}
}
}
