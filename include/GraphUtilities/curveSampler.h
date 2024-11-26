#pragma once

// Name inspired from https://github.com/FreyaHolmer/Mathfs/blob/master/Runtime/Splines/UniformCurveSampler.cs

#include "GraphUtilities/uniformCubicSpline.h"
#include <vector>


// Class

class CurveSampler {
public:
	CurveSampler();
	CurveSampler(UniformCubicSpline &spline);

	void _onInit();

	void setUniformCubicSpline(UniformCubicSpline &spline);
	std::vector<double> _getCurvePosition(double t);

	void calculateByResolution(double t_start, double t_end, int resolution = 30);

	void calculateByBisection(double t_start, double t_end, double maxDistance = 0.01);
	double _bisectionRecursion(double maxDistance, double cumulativeDistance, double t_start, double t_end, std::vector<double> startPosition, std::vector<double> endPosition);

	double paramToDistance(double t);

	double distanceToParam(double distance);

private:
	std::vector<std::pair<double, double>> t_cumulativeDistances;
	UniformCubicSpline spline;
};
