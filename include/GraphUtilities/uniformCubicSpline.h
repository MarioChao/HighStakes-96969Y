#pragma once

#include "GraphUtilities/cubicSplineSegment.h"

#include <vector>


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

	CubicSplineSegment &getSegment(int id);
	std::vector<double> getPositionAtT(double t);
	std::vector<double> getVelocityAtT(double t);

	std::pair<double, double> getTRange();

private:
	std::vector<CubicSplineSegment> segments;
};
