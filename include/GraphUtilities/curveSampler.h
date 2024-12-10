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

	// Configure spline
	void setUniformCubicSpline(UniformCubicSpline &spline);
	std::vector<double> _getCurvePosition(double t);

	// Preprocess the spline to enable sampling
	CurveSampler &calculateByResolution(int resolution = 30);

	// Spline data
	std::pair<double, double> getTRange();
	std::pair<double, double> getDistanceRange();
	UniformCubicSpline getSpline();

	// Sampling
	double paramToDistance(double t);
	double distanceToParam(double distance);

private:
	std::vector<std::pair<double, double>> t_cumulativeDistances;
	UniformCubicSpline spline;
};
