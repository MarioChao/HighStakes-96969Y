#include "Pas1-Lib/Planning/Trajectories/curvature.h"

#include "Aespa-Lib/Winter-Utilities/general.h"
#include <cmath>


namespace pas1_lib {
namespace planning {
namespace trajectories {


// ---------- Curvature Point ----------

void CurvaturePoint::maxSmooth(CurvaturePoint &previousPoint, double epsilon) {
	// See https://en.wikipedia.org/wiki/Smooth_maximum
	double d1 = previousPoint.distance;
	double d2 = distance;
	double k1 = previousPoint.curvature;
	double k2 = curvature;
	int sign1 = aespa_lib::genutil::signum(k1);
	int sign2 = aespa_lib::genutil::signum(k2);
	if (
		std::fabs(k2) < std::fabs(k1)
		&& sign1 == sign2
	) {
		double e = std::fmin(epsilon * std::pow(d1 - d2, 2), std::pow(k1 - k2, 2));
		double maxK = (k1 + k2 + sign1 * std::sqrt(std::pow(k1 - k2, 2) - e)) / 2;
		curvature = maxK;
	}
}


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
	if (points.empty()) return 0;

	// Binary search
	int bs_l, bs_r, bs_m;
	int bs_result = 0;
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

double CurvatureSequence::getControlPointDistance(double distance, bool nextPoint) {
	sort();
	if (points.empty()) return distance;

	// Binary search
	int bs_l, bs_r, bs_m;
	int bs_result = 0;
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
	if (nextPoint) bs_result++;
	if (bs_result >= (int) points.size()) return distance;

	return points[bs_result].distance;
}

void CurvatureSequence::sort() {
	if (!isSorted) {
		std::sort(points.begin(), points.end(), [&](CurvaturePoint a, CurvaturePoint b) {
			return a.distance < b.distance;
		});
		isSorted = true;
	}
}

void CurvatureSequence::maxSmooth(double epsilon) {
	sort();

	// Forward and backward max smoothing
	int pointsCount = points.size();
	for (int i = 0; i < pointsCount - 1; i++) {
		CurvaturePoint &point1 = points[i];
		CurvaturePoint &point2 = points[i + 1];
		point2.maxSmooth(point1, epsilon);
	}
	for (int i = pointsCount - 1; i > 0; i--) {
		CurvaturePoint &point1 = points[i];
		CurvaturePoint &point2 = points[i - 1];
		point2.maxSmooth(point1, epsilon);
	}
}


}
}
}
