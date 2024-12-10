#pragma once

#include "GraphUtilities/cubicSplineSegment.h"

#include <initializer_list>
#include <vector>


// Forward declaration

class Linegular;


// Class

class UniformCubicSpline {
public:
	UniformCubicSpline();
	UniformCubicSpline(std::vector<CubicSplineSegment> segments);

	/// @brief Creates a spline with the given points. Only use B-Spline or Catmull-Rom.
	static UniformCubicSpline fromAutoTangent(cspline::SplineType splineType, std::vector<std::vector<double>> points);

	/// @brief Extends the spline by adding a new segment. Only use for B-Spline or Catmull-Rom.
	UniformCubicSpline &extendPoint(std::vector<double> newPoint);

	/// @brief Extends the spline by adding new segments. Only use for B-Spline or Catmull-Rom.
	UniformCubicSpline &extendPoints(std::vector<std::vector<double>> newPoints);

	UniformCubicSpline &attachSegment(CubicSplineSegment newSegment);

	std::vector<CubicSplineSegment> getSegments();

	CubicSplineSegment &getSegment(int id);
	std::vector<double> getPositionAtT(double t);
	std::vector<double> getVelocityAtT(double t);
	double getPolarAngleRadiansAt(double t);

	std::vector<double> getSecondPrimeAtT(double t);

	double getCurvatureAt(double t);

	std::pair<double, double> getTRange();

	UniformCubicSpline getReversed();

	Linegular getLinegularAt(double t, bool reverseHeading = false);

private:
	std::vector<CubicSplineSegment> segments;
	static CubicSplineSegment emptySegment;
};
