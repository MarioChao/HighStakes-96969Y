#include "GraphUtilities/uniformCubicSpline.h"

#include <cmath>

UniformCubicSpline::UniformCubicSpline() {
	this->segments.clear();
}

UniformCubicSpline::UniformCubicSpline(std::vector<CubicSplineSegment> segments) {
	this->segments = segments;
}

void UniformCubicSpline::extendPoint(std::vector<double> newPoint) {
	CubicSplineSegment lastSegment = getSegment((int) segments.size() - 1);
	std::vector<std::vector<double>> points = lastSegment.getControlPoints();
	this->segments.push_back(CubicSplineSegment(
		lastSegment.getSplineType(),
		{
			points[1], points[2], points[3], newPoint
		}
	));
}

CubicSplineSegment &UniformCubicSpline::getSegment(int id) {
	// Validate
	if (!(0 <= id && id < (int) segments.size())) {
		CubicSplineSegment emptySegment;
		return emptySegment;
	}

	// Return result
	return segments[id];
}

std::vector<double> UniformCubicSpline::getPositionAtT(double t) {
	// Get segment info
	int segment_id = floor(t);
	double segment_t = t - segment_id;

	// Special cases
	if (segment_id < 0) {
		return segments[0].getPositionAtT(0);
	} else if (segment_id >= (int) segments.size()) {
		return segments.back().getPositionAtT(1);
	}
	return segments[segment_id].getPositionAtT(segment_t);
}

std::vector<double> UniformCubicSpline::getVelocityAtT(double t) {
	// Get segment info
	int segment_id = floor(t);
	double segment_t = t - segment_id;

	// Special cases
	if (segment_id < 0) {
		return segments[0].getVelocityAtT(0);
	} else if (segment_id >= (int) segments.size()) {
		return segments.back().getVelocityAtT(1);
	}
	return segments[segment_id].getVelocityAtT(segment_t);
}

std::pair<double, double> UniformCubicSpline::getTRange() {
	return std::make_pair(0, (int) segments.size());
}
