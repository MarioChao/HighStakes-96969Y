#include "GraphUtilities/cubicSplineSegment.h"

#include <stdio.h>

CubicSplineSegment::CubicSplineSegment() {
	setSplineType(cspline::SplineType::Bezier);
	setPoints(std::vector<std::vector<double>>(4, std::vector<double>(2)));
}

CubicSplineSegment::CubicSplineSegment(cspline::SplineType splineType, std::vector<std::vector<double>> points) {
	setSplineType(splineType);
	setPoints(points);
}

void CubicSplineSegment::setSplineType(cspline::SplineType splineType) {
	if (splineType == this->splineType) {
		return;
	}

	this->splineType = splineType;
}

void CubicSplineSegment::setPoints(std::vector<std::vector<double>> points) {
	control_points = points;
	Matrix matrix_data(points);
	stored_points = getStoringMatrix().multiply(matrix_data).data;
}

cspline::SplineType CubicSplineSegment::getSplineType() {
	return splineType;
}

std::vector<std::vector<double>> CubicSplineSegment::getControlPoints() {
	return control_points;
}

Matrix &CubicSplineSegment::getCharacteristicMatrix() {
	switch (splineType) {
		case cspline::Bezier:
			return cspline::characteristic_matrix::Bezier;
		case cspline::Hermite:
			return cspline::characteristic_matrix::Hermite;
		case cspline::CatmullRom:
			return cspline::characteristic_matrix::CatmullRom;
		case cspline::B_Spline:
			return cspline::characteristic_matrix::B_Spline;
	}
}

Matrix &CubicSplineSegment::getStoringMatrix() {
	switch (splineType) {
		case cspline::Bezier:
			return cspline::storing_matrix::Bezier;
		case cspline::Hermite:
			return cspline::storing_matrix::Hermite;
		case cspline::CatmullRom:
			return cspline::storing_matrix::CatmullRom;
		case cspline::B_Spline:
			return cspline::storing_matrix::B_Spline;
	}
}

std::vector<double> CubicSplineSegment::getPositionAtT(double t) {
	Matrix t_matrix({ {1, t, t*t, t*t*t} });
	Matrix point_matrix = Matrix(stored_points);
	std::vector<double> point = t_matrix.multiply(getCharacteristicMatrix()).multiply(point_matrix).data[0];
	return point;
}

std::vector<double> CubicSplineSegment::getVelocityAtT(double t) {
	Matrix t_matrix({ {0, 1, 2*t, 3*t*t} });
	Matrix point_matrix = Matrix(stored_points);
	std::vector<double> point = t_matrix.multiply(getCharacteristicMatrix()).multiply(point_matrix).data[0];
	return point;
}

std::vector<double> CubicSplineSegment::getSecondPrimeAtT(double t) {
	Matrix t_matrix({ {0, 0, 2, 6*t} });
	Matrix point_matrix = Matrix(stored_points);
	std::vector<double> point = t_matrix.multiply(getCharacteristicMatrix()).multiply(point_matrix).data[0];
	return point;
}

CubicSplineSegment CubicSplineSegment::getReversed() {
	// Create new segment of same type
	CubicSplineSegment resultSegment;
	resultSegment.setSplineType(splineType);

	// Set reversed control points
	std::vector<std::vector<double>> newControlPoints = control_points;
	std::reverse(newControlPoints.begin(), newControlPoints.end());
	resultSegment.setPoints(newControlPoints);

	// Return segment
	return resultSegment;
}

namespace cspline {
	namespace characteristic_matrix {
		Matrix Bezier = Matrix({
			{1, 0, 0, 0},
			{-3, 3, 0, 0},
			{3, -6, 3, 0},
			{-1, 3, -3, 1},
		});
		Matrix Hermite = Matrix({
			{1, 0, 0, 0},
			{0, 1, 0, 0},
			{-3, -2, 3, -1},
			{2, 1, -2, 1},
		});
		Matrix CatmullRom = Matrix({
			{0, 2, 0, 0},
			{-1, 0, 1, 0},
			{2, -5, 4, -1},
			{-1, 3, -3, 1},
		}) * 0.5;
		Matrix B_Spline = Matrix({
			{1, 4, 1, 0},
			{-3, 0, 3, 0},
			{3, -6, 3, 0},
			{-1, 3, -3, 1},
		}) * (1.0 / 6.0);
	}

	namespace storing_matrix {
		Matrix Bezier = Matrix::identity(4);
		Matrix Hermite = Matrix({
			{1, 0, 0, 0},
			{-1, 1, 0, 0},
			{0, 0, 1, 0},
			{0, 0, -1, 1},
		});
		Matrix CatmullRom = Matrix::identity(4);
		Matrix B_Spline = Matrix::identity(4);
	}
}
