#include "GraphUtilities/curveSampler.h"

#include "Utilities/generalUtility.h"

#include <stdio.h>

CurveSampler::CurveSampler() {
	_onInit();
}

CurveSampler::CurveSampler(UniformCubicSpline &spline) {
	setUniformCubicSpline(spline);
	_onInit();
}

void CurveSampler::_onInit() {
	t_cumulativeDistances.clear();
}

void CurveSampler::setUniformCubicSpline(UniformCubicSpline &spline) {
	this->spline = spline;
}

std::vector<double> CurveSampler::_getCurvePosition(double t) {
	return spline.getPositionAtT(t);
}

CurveSampler &CurveSampler::calculateByResolution(int resolution) {
	// Get t interval
	std::pair<double, double> tRange = spline.getTRange();
	double t_start = tRange.first;
	double t_end = tRange.second;

	t_cumulativeDistances.clear();
	t_cumulativeDistances.push_back(std::make_pair(t_start, 0));

	// Initialize variables
	std::vector<double> previousPoint, currentPoint;
	previousPoint = _getCurvePosition(t_start);
	double pathLength = 0;

	// Look through each segment
	for (int i = 1; i <= resolution; i++) {
		// Get spline point
		double t = genutil::rangeMap(i, 0, resolution, t_start, t_end);
		currentPoint = _getCurvePosition(t);

		// Add segment length to path length
		double segmentLength = genutil::euclideanDistance(previousPoint, currentPoint);
		pathLength += segmentLength;

		// Store distance
		t_cumulativeDistances.push_back(std::make_pair(t, pathLength));

		// Update
		previousPoint = currentPoint;
	}

	// Method chaining
	return *this;
}

std::pair<double, double> CurveSampler::getTRange() {
	return spline.getTRange();
}

std::pair<double, double> CurveSampler::getDistanceRange() {
	return std::make_pair(t_cumulativeDistances.front().second, t_cumulativeDistances.back().second);
}

UniformCubicSpline CurveSampler::getSpline() {
	return spline;
}

double CurveSampler::paramToDistance(double t) {
	// Check extreme
	if (t <= t_cumulativeDistances.front().first) {
		return t_cumulativeDistances.front().second;
	}
	if (t >= t_cumulativeDistances.back().first) {
		return t_cumulativeDistances.back().second;
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
		std::pair<double, double> t_distance1 = t_cumulativeDistances[bM1];
		std::pair<double, double> t_distance2 = t_cumulativeDistances[bM2];
		bool isInRange = (
			t_distance1.first <= t
			&& t <= t_distance2.first
		);

		// Update endpoints
		if (isInRange) {
			return genutil::rangeMap(
				t,
				t_distance1.first, t_distance2.first,
				t_distance1.second, t_distance2.second
			);
		} else if (t < t_distance1.first) {
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
	if (distance <= t_cumulativeDistances.front().second) {
		return t_cumulativeDistances.front().first;
	}
	if (distance >= t_cumulativeDistances.back().second) {
		return t_cumulativeDistances.back().first;
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
		std::pair<double, double> t_distance1 = t_cumulativeDistances[bM1];
		std::pair<double, double> t_distance2 = t_cumulativeDistances[bM2];
		bool isInRange = (
			t_distance1.second <= distance
			&& distance <= t_distance2.second
		);

		// Update endpoints
		if (isInRange) {
			return genutil::rangeMap(
				distance,
				t_distance1.second, t_distance2.second,
				t_distance1.first, t_distance2.first
			);
		} else if (distance < t_distance1.second) {
			bR = bM1 - 1;
		} else {
			bL = bM1 + 1;
		}
	}

	// Return 0 if fails
	return 0;
}
