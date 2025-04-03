#include "Pas1-Lib/Planning/Splines/spline-curve.h"

#include "Aespa-Lib/Karina-Data-Structures/linegular.h"
#include "Aespa-Lib/Winter-Utilities/general.h"
#include <cmath>
#include <algorithm>
#include <stdio.h>


namespace {
using aespa_lib::datas::Linegular;
using aespa_lib::units::PolarAngle;
using namespace aespa_lib::units::literals;
using namespace pas1_lib::planning::segments;
}


namespace pas1_lib {
namespace planning {
namespace splines {


SplineCurve::SplineCurve(std::vector<std::shared_ptr<SegmentBase>> segments)
	: segments(segments) {}

SplineCurve::SplineCurve() {
	this->segments.clear();
}

SplineCurve SplineCurve::fromAutoTangent_cubicSpline(
	SplineType splineType, std::vector<std::vector<double>> points,
	double knot_parameter_alpha
) {
	// Validate input
	if ((int) points.size() < 4) {
		// return SplineCurve();
		printf("ERR: Spline points not enough\n");
		throw;
	}

	// Create spline
	auto extendedPoints = std::vector<std::vector<double>>(points.begin() + 4, points.end());
	SplineCurve spline = SplineCurve()
		.attachSegment(std::shared_ptr<SegmentBase>(
			new CubicSplineSegment(splineType, { points[0], points[1], points[2], points[3] }, knot_parameter_alpha)
		))
		.extendPoints_cubicSpline(extendedPoints);

	// Return
	return spline;
}

SplineCurve &SplineCurve::extendPoint_cubicSpline(std::vector<double> &newPoint) {
	SegmentBase &lastSegment = getSegment((int) segments.size() - 1);
	std::vector<std::vector<double>> points = lastSegment.getControlPoints();
	attachSegment(std::shared_ptr<SegmentBase>(
		new CubicSplineSegment(lastSegment.getSplineType(), { points[1], points[2], points[3], newPoint }, lastSegment.knot_parameter_alpha)
	));

	// Method chaining
	return *this;
}

SplineCurve &SplineCurve::extendPoints_cubicSpline(std::vector<std::vector<double>> &newPoints) {
	for (std::vector<double> &point : newPoints) {
		extendPoint_cubicSpline(point);
	}

	// Method chaining
	return *this;
}

SplineCurve &SplineCurve::attachSegment(std::shared_ptr<SegmentBase> newSegment) {
	this->segments.push_back(newSegment);

	// Method chaining
	return *this;
}

std::vector<std::shared_ptr<SegmentBase>> SplineCurve::getSegments() {
	return segments;
}

SegmentBase &SplineCurve::getSegment(int id) {
	// Validate
	if (!(0 <= id && id < (int) segments.size())) {
		printf("ERR: Id not valid\n");
		throw;
	}

	// Return result
	return *segments[id];
}

std::pair<int, double> SplineCurve::getSegmentIndex(double t) {
	// Segment info
	int segment_id = floor(t);
	double segment_t = t - segment_id;

	// Special cases
	if (segment_id < 0) {
		return {0, 0};
	} else if (segment_id >= (int) segments.size()) {
		return {(int) segments.size() - 1, 1};
	}

	// Normal case
	return {segment_id, segment_t};
}

std::vector<double> SplineCurve::getPositionAtT(double t) {
	// Get segment info
	auto segmentIndex = getSegmentIndex(t);
	return segments[segmentIndex.first]->getPositionAtT(segmentIndex.second);
}

std::vector<double> SplineCurve::getFirstPrimeAtT(double t) {
	// Get segment info
	auto segmentIndex = getSegmentIndex(t);
	return segments[segmentIndex.first]->getFirstPrimeAtT(segmentIndex.second);
}

std::vector<double> SplineCurve::getSecondPrimeAtT(double t) {
	// Get segment info
	auto segmentIndex = getSegmentIndex(t);
	return segments[segmentIndex.first]->getSecondPrimeAtT(segmentIndex.second);
}

PolarAngle SplineCurve::getPolarAngleAt(double t) {
	std::vector<double> velocity = getFirstPrimeAtT(t);
	return operator ""_polarRad((long double) atan2(velocity[1], velocity[0]));
}

double SplineCurve::getCurvatureAt(double t) {
	std::vector<double> prime1 = getFirstPrimeAtT(t);
	std::vector<double> prime2 = getSecondPrimeAtT(t);

	// See https://en.wikipedia.org/wiki/Curvature#In_terms_of_a_general_parametrization

	double xp = prime1[0], xpp = prime2[0];
	double yp = prime1[1], ypp = prime2[1];

	double k = (
		(xp * ypp - yp * xpp)
		/ std::pow(xp * xp + yp * yp, 3.0 / 2.0)
	);
	return k;
}

std::pair<double, double> SplineCurve::getTRange() {
	return std::make_pair(0, (int) segments.size());
}

SplineCurve SplineCurve::getReversed() {
	// Create new spline
	SplineCurve resultSpline;

	// Append reversed segments
	for (std::shared_ptr<SegmentBase> &segment : this->segments) {
		resultSpline.attachSegment(segment->getReversed());
	}

	// Reverse segments order
	std::reverse(resultSpline.segments.begin(), resultSpline.segments.end());

	// Return spline
	return resultSpline;
}

Linegular SplineCurve::getLinegularAt(double t, bool reverseHeading) {
	// Get position
	std::vector<double> position = getPositionAtT(t);

	// Get rotation
	PolarAngle rotation = getPolarAngleAt(t);
	rotation += reverseHeading * 180.0;
	rotation = aespa_lib::genutil::modRange(rotation.polarDeg(), 360, -180);

	// Create and return linegular
	Linegular lg(position[0], position[1], rotation);
	return lg;
}


}
}
}
