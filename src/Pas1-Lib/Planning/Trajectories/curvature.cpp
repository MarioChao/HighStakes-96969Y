#include "Pas1-Lib/Planning/Trajectories/curvature.h"

#include "Aespa-Lib/Winter-Utilities/general.h"
#include <cmath>


namespace pas1_lib {
namespace planning {
namespace trajectories {


// ---------- Curvature Sequence ----------

CurvaturePoint::CurvaturePoint(double distance, double curvature)
	: distance(distance), curvature(curvature) {}

CurvatureSequence::CurvatureSequence()
	: isSorted(false) {};

void CurvatureSequence::addPoint(double distance, double curvature) {
	points.push_back(CurvaturePoint(distance, curvature));
	isSorted = false;
}

double CurvatureSequence::getCurvatureAtDistance(double distance) {
	sort();

	// Binary search
	int bs_l, bs_r, bs_m;
	int bs_result = -1;
	bs_l = 0;
	bs_r = (int) points.size() - 1;
	while (bs_l <= bs_r) {
		bs_m = bs_l + (bs_r - bs_l) / 2;
		CurvaturePoint &point = points[bs_m];
		if (point.distance <= distance + 1e-5) {
			bs_result = bs_m;
			bs_l = bs_m + 1;
		} else {
			bs_r = bs_m - 1;
		}
	}
	if (bs_result == -1) return 0;

	// Lerp
	double result = points[bs_result].curvature;
	if (bs_result + 1 < (int) points.size()) {
		CurvaturePoint &thisPoint = points[bs_result];
		CurvaturePoint &nextPoint = points[bs_result + 1];
		result = aespa_lib::genutil::rangeMap(
			distance, thisPoint.distance, nextPoint.distance,
			thisPoint.curvature, nextPoint.curvature
		);
	}

	return result;
}

void CurvatureSequence::sort() {
	if (!isSorted) {
		std::sort(points.begin(), points.end(), [&](CurvaturePoint a, CurvaturePoint b) {
			return a.distance < b.distance;
		});
		isSorted = true;
	}
}

void CurvatureSequence::smoothen(double alpha) {
	// Apply smoothing with EMA
	double newCurvature = 0;
	for (CurvaturePoint &point : points) {
		double k = point.curvature;
		newCurvature = alpha * k + (1 - alpha) * newCurvature;
		point.curvature = newCurvature;
	}
}


}
}
}
