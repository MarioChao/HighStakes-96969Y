#pragma once

#include <vector>
#include <memory>
#include "Aespa-Lib/Karina-Data-Structures/linegular.h"
#include "Aespa-Lib/Giselle-Geometry/polygon-2d.hpp"


namespace pas1_lib {
namespace planning {
namespace trajectories {


// ---------- Trajectory Constraint ----------

struct TrajectoryConstraint {
	virtual double calculateMaxVelocity(aespa_lib::datas::Linegular pose, double curvature, double velocity);
};

double getVelocity_trajectoryConstraints(std::vector<std::shared_ptr<TrajectoryConstraint>> constraints, aespa_lib::datas::Linegular pose, double curvature, double velocity);


struct CentripetalAccelerationConstraint : public TrajectoryConstraint {
	CentripetalAccelerationConstraint(double maxCentripetalAcceleration);

	double calculateMaxVelocity(aespa_lib::datas::Linegular pose, double curvature, double velocity) override;


	double maxCentripetalAcceleration;
};


struct PolygonRegionConstraint : public TrajectoryConstraint {
	PolygonRegionConstraint(aespa_lib::geometry::Polygon2D polygon, double maxVelocity);

	double calculateMaxVelocity(aespa_lib::datas::Linegular pose, double curvature, double velocity) override;


	aespa_lib::geometry::Polygon2D polygon;
	double maxVelocity;
};


}
}
}
