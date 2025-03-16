#include "Pas1-Lib/Planning/Splines/curve-sampler.h"

#include "Aespa-Lib/Winter-Utilities/general.h"

#include <stdio.h>


namespace pas1_lib {
namespace planning {
namespace splines {


// ---------- Struct ----------

CurveParam::CurveParam(double t, double distance)
	: t(t), distance(distance) {};


// ---------- Class ----------

CurveSampler::CurveSampler(SplineCurve spline)
	: spline(std::shared_ptr<SplineCurve>(new SplineCurve(spline))) {
	reset();
}

CurveSampler::CurveSampler()
	: CurveSampler(SplineCurve()) {}

void CurveSampler::reset() {
	t_cumulativeDistances.clear();
}

void CurveSampler::setSpline(std::shared_ptr<SplineCurve> spline) {
	this->spline = spline;
}

std::vector<double> CurveSampler::_getCurvePosition(double t) {
	return spline->getPositionAtT(t);
}

std::vector<double> CurveSampler::_getCurveFirstPrime(double t) {
	return spline->getFirstPrimeAtT(t);
}

CurveSampler &CurveSampler::calculateByResolution(int resolution) {
	// Get t interval
	std::pair<double, double> tRange = spline->getTRange();
	double t_start = tRange.first;
	double t_end = tRange.second;

	t_cumulativeDistances.clear();
	t_cumulativeDistances.push_back(CurveParam(t_start, 0));

	// Initialize variables
	std::vector<double> previousPoint, currentPoint;
	previousPoint = _getCurvePosition(t_start);
	double previous_1Prime, current_1Prime, middle_1Prime;
	previous_1Prime = aespa_lib::genutil::l2Norm(_getCurveFirstPrime(t_start));
	double previousT = 0;

	double pathLength = 0;

	// Look through each segment
	for (int i = 1; i <= resolution; i++) {
		// Get spline time
		double t = aespa_lib::genutil::rangeMap(i, 0, resolution, t_start, t_end);
		double deltaT = t - previousT;

		// Get point info
		currentPoint = _getCurvePosition(t);
		current_1Prime = aespa_lib::genutil::l2Norm(_getCurveFirstPrime(t));
		middle_1Prime = aespa_lib::genutil::l2Norm(_getCurveFirstPrime(t - deltaT / 2));

		// Compute segment length
		double segmentLength;

		// Arc length formula: √[(dx/dt)^2 + (dy/dt)^2]dt
		switch (4) {
			case 2:
				// Method 2: trapezoidal sum
				segmentLength = (previous_1Prime + current_1Prime) * deltaT / 2;
				break;
			case 3:
				// Method 3: midpoint sum
				segmentLength = middle_1Prime * deltaT;
				break;
			case 4:
				// Method 4: Simpson's rule
				segmentLength = (previous_1Prime + 4 * middle_1Prime + current_1Prime) * deltaT / 6;
				break;
			default:
				// Method 1: euclidean distance: √(dx^2 + dy^2)
				segmentLength = aespa_lib::genutil::euclideanDistance(previousPoint, currentPoint);
				break;
		}

		// Add segment length to path length
		pathLength += segmentLength;

		// Store distance
		t_cumulativeDistances.push_back(CurveParam(t, pathLength));

		// Update
		previousPoint = currentPoint;
		previous_1Prime = current_1Prime;
		previousT = t;
	}

	// Method chaining
	return *this;
}

std::pair<double, double> CurveSampler::getTRange() {
	return spline->getTRange();
}

std::pair<double, double> CurveSampler::getDistanceRange() {
	return std::make_pair(t_cumulativeDistances.front().distance, t_cumulativeDistances.back().distance);
}

SplineCurve &CurveSampler::getSpline() {
	return *spline;
}

double CurveSampler::paramToDistance(double t) {
	// Check extreme
	if (t <= t_cumulativeDistances.front().t) {
		return t_cumulativeDistances.front().distance;
	}
	if (t >= t_cumulativeDistances.back().t) {
		return t_cumulativeDistances.back().distance;
	}

	// Binary search for t
	int bL, bR;
	bL = 0;
	bR = (int) t_cumulativeDistances.size() - 2;
	while (bL <= bR) {
		// Get midpoint
		int bM1 = bL + (bR - bL) / 2;
		int bM2 = bM1 + 1;

		// Check if value is in range
		CurveParam t_distance1 = t_cumulativeDistances[bM1];
		CurveParam t_distance2 = t_cumulativeDistances[bM2];
		bool isInRange = (
			t_distance1.t <= t
			&& t <= t_distance2.t
		);

		// Update endpoints
		if (isInRange) {
			return aespa_lib::genutil::rangeMap(
				t,
				t_distance1.t, t_distance2.t,
				t_distance1.distance, t_distance2.distance
			);
		} else if (t < t_distance1.t) {
			bL = bM1 + 1;
		} else {
			bR = bM1 - 1;
		}
	}

	// Return 0 if fails
	return 0;
}

double CurveSampler::distanceToParam(double distance) {
	// Check extreme
	if (distance <= t_cumulativeDistances.front().distance) {
		return t_cumulativeDistances.front().t;
	}
	if (distance >= t_cumulativeDistances.back().distance) {
		return t_cumulativeDistances.back().t;
	}

	// Binary search for distance
	int bL, bR;
	bL = 0;
	bR = (int) t_cumulativeDistances.size() - 2;
	while (bL <= bR) {
		// Get midpoint
		int bM1 = bL + (bR - bL) / 2;
		int bM2 = bM1 + 1;

		// Check if value is in range
		CurveParam t_distance1 = t_cumulativeDistances[bM1];
		CurveParam t_distance2 = t_cumulativeDistances[bM2];
		bool isInRange = (
			t_distance1.distance <= distance
			&& distance <= t_distance2.distance
		);

		// Update endpoints
		if (isInRange) {
			return aespa_lib::genutil::rangeMap(
				distance,
				t_distance1.distance, t_distance2.distance,
				t_distance1.t, t_distance2.t
			);
		} else if (distance < t_distance1.distance) {
			bR = bM1 - 1;
		} else {
			bL = bM1 + 1;
		}
	}

	// Return 0 if fails
	return 0;
}


}
}
}
