#pragma once

#include "GraphUtilities/cubicSplineSegment.h"

#include <vector>


// Class

class UniformCubicSpline {
public:
	UniformCubicSpline(std::vector<CubicSplineSegment> segments);

	/**
	 * @brief Extends the spline by adding a new segment.
	 * Only use for B-Spline or Catmull-Rom.
	 * 
	 */
	void extendPoint(std::vector<double> newPoint);

	CubicSplineSegment &getSegment(int id);
	std::pair<double, double> getPositionAtT(double t);
	std::pair<double, double> getVelocityAtT(double t);

private:
	std::vector<CubicSplineSegment> segments;
};
