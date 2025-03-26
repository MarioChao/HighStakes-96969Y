#pragma once

#include <algorithm>
#include <vector>


namespace pas1_lib {
namespace planning {
namespace trajectories {


// ---------- Curvature Sequence ----------

struct CurvaturePoint {
	CurvaturePoint(double distance, double curvature);

	double distance, curvature;
};

struct CurvatureSequence {
	CurvatureSequence();

	void addPoint(double distance, double curvature);

	double getCurvatureAtDistance(double distance);
	double getControlPointDistance(double distance, bool nextPoint = true);

	void sort();
	void smoothen(double alpha);


	std::vector<CurvaturePoint> points;

	bool isSorted;
};


}
}
}
