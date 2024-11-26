#pragma once

#include "GraphUtilities/cubicSplineSegment.h"

#include <vector>


// Forward declaration

class Linegular;


// Class

class UniformCubicSpline {
public:
	UniformCubicSpline();
	UniformCubicSpline(std::vector<CubicSplineSegment> segments);

	/**
	 * @brief Extends the spline by adding a new segment.
	 * Only use for B-Spline or Catmull-Rom.
	 * 
	 */
	void extendPoint(std::vector<double> newPoint);

	void attachSegment(CubicSplineSegment newSegment);

	CubicSplineSegment &getSegment(int id);
	std::vector<double> getPositionAtT(double t);
	std::vector<double> getVelocityAtT(double t);
	double getPolarAngleRadiansAt(double t);

	std::pair<double, double> getTRange();

	UniformCubicSpline getReversed();

	Linegular getLinegularAt(double t, bool reverseHeading = false);

private:
	std::vector<CubicSplineSegment> segments;
};
