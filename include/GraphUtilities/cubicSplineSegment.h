#pragma once

#include "GraphUtilities/matrix.h"

#include <utility>
#include <vector>


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

	void setPoints(std::vector<std::vector<double>> points);

	Matrix &getCharacteristicMatrix();
	Matrix &getStoringMatrix();

	std::pair<double, double> getPositionAtT(double t);
	std::pair<double, double> getVelocityAtT(double t);

private:
	cspline::SplineType splineType;

	std::vector<std::vector<double>> stored_points;
	std::vector<std::vector<double>> control_points;
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
