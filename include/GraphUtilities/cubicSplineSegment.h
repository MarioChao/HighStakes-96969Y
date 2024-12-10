#pragma once

#include "GraphUtilities/matrix.h"

#include <utility>
#include <vector>
#include <algorithm>


// Enumeration

namespace cspline {
	enum SplineType {
		Bezier,
		Hermite,
		CatmullRom,
		B_Spline,
	};
}


// Class

class CubicSplineSegment {
public:
	CubicSplineSegment();
	CubicSplineSegment(cspline::SplineType splineType, std::vector<std::vector<double>> points);

	void setSplineType(cspline::SplineType splineType);
	void setPoints(std::vector<std::vector<double>> points);

	cspline::SplineType getSplineType();
	std::vector<std::vector<double>> getControlPoints();

	Matrix &getCharacteristicMatrix();
	Matrix &getStoringMatrix();

	std::vector<double> getPositionAtT(double t);
	std::vector<double> getVelocityAtT(double t);
	std::vector<double> getSecondPrimeAtT(double t);

	CubicSplineSegment getReversed();

private:
	cspline::SplineType splineType;

	std::vector<std::vector<double>> control_points;
	std::vector<std::vector<double>> stored_points;
};

namespace cspline {
	namespace characteristic_matrix {
		extern Matrix Bezier;
		extern Matrix Hermite;
		extern Matrix CatmullRom;
		extern Matrix B_Spline;
	}
	namespace storing_matrix {
		extern Matrix Bezier;
		extern Matrix Hermite;
		extern Matrix CatmullRom;
		extern Matrix B_Spline;
	}
}
