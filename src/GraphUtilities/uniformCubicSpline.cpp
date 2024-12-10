#include "GraphUtilities/uniformCubicSpline.h"

#include "AutonUtilities/linegular.h"
#include "Utilities/generalUtility.h"
#include <cmath>
#include <algorithm>
#include <stdio.h>

UniformCubicSpline::UniformCubicSpline() {
	this->segments.clear();
}

UniformCubicSpline::UniformCubicSpline(std::vector<CubicSplineSegment> segments) {
	this->segments = segments;
}

UniformCubicSpline UniformCubicSpline::fromAutoTangent(cspline::SplineType splineType, std::vector<std::vector<double>> points) {
	// Validate input
	if ((int) points.size() < 4) {
		return UniformCubicSpline();
	}

	// Create spline
	UniformCubicSpline spline = UniformCubicSpline()
	.attachSegment(CubicSplineSegment(splineType, {points[0], points[1], points[2], points[3]}))
	.extendPoints(std::vector<std::vector<double>>(points.begin() + 4, points.end()));

	// Return
	return spline;
}

UniformCubicSpline &UniformCubicSpline::extendPoint(std::vector<double> newPoint) {
	CubicSplineSegment lastSegment = getSegment((int) segments.size() - 1);
	std::vector<std::vector<double>> points = lastSegment.getControlPoints();
	attachSegment(CubicSplineSegment(lastSegment.getSplineType(), {points[1], points[2], points[3], newPoint}));

	// Method chaining
	return *this;
}

UniformCubicSpline &UniformCubicSpline::extendPoints(std::vector<std::vector<double>> newPoints) {
	for (std::vector<double> point : newPoints) {
		extendPoint(point);
	}

	// Method chaining
	return *this;
}

UniformCubicSpline &UniformCubicSpline::attachSegment(CubicSplineSegment newSegment) {
	this->segments.push_back(newSegment);

	// Method chaining
	return *this;
}

CubicSplineSegment &UniformCubicSpline::getSegment(int id) {
	// Validate
	if (!(0 <= id && id < (int) segments.size())) {
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

double UniformCubicSpline::getPolarAngleRadiansAt(double t) {
	std::vector<double> velocity = getVelocityAtT(t);
	return atan2(velocity[1], velocity[0]);
}

std::pair<double, double> UniformCubicSpline::getTRange() {
	return std::make_pair(0, (int) segments.size());
}

UniformCubicSpline UniformCubicSpline::getReversed() {
	// Create new spline
	UniformCubicSpline resultSpline;

	// Append reversed segments
	for (CubicSplineSegment &segment : this->segments) {
		resultSpline.attachSegment(segment.getReversed());
	}

	// Reverse segments order
	std::reverse(resultSpline.segments.begin(), resultSpline.segments.end());

	// Return spline
	return resultSpline;
}

Linegular UniformCubicSpline::getLinegularAt(double t, bool reverseHeading) {
	// Get position
	std::vector<double> position = getPositionAtT(t);

	// Get angle
	double angle_radians = getPolarAngleRadiansAt(t);
	double finalAngle_degrees = genutil::toDegrees(angle_radians) + reverseHeading * 180.0;
	finalAngle_degrees = genutil::modRange(finalAngle_degrees, 360, -180);

	// Create and return linegular
	Linegular lg(position[0], position[1], finalAngle_degrees);
	return lg;
}

CubicSplineSegment UniformCubicSpline::emptySegment;
