#include "Pas1-Lib/Planning/Segments/cubic-spline.h"

#include "Aespa-Lib/Winter-Utilities/general.h"
#include <cmath>


namespace {
using aespa_lib::datas::Matrix;
}


namespace pas1_lib {
namespace planning {
namespace segments {


// ---------- Constants ----------

namespace characteristic_matrix {
Matrix Cubic_Hermite(0, 0);
}

namespace storing_matrix {
Matrix Cubic_Bezier(0, 0);
Matrix Cubic_Hermite(0, 0);
Matrix CatmullRom(0, 0);
Matrix Cubic_B_Spline(0, 0);
}

void setMatrices() {
	if (characteristic_matrix::Cubic_Hermite.getShape().first == 0) {
		characteristic_matrix::Cubic_Hermite = Matrix({
			{1, 0, 0, 0},
			{0, 0, 1, 0},
			{-3, 3, -2, -1},
			{2, -2, 1, 1},
		});
	}

	if (storing_matrix::Cubic_Bezier.getShape().first == 0) {
		storing_matrix::Cubic_Bezier = Matrix({
			{1, 0, 0, 0},
			{0, 0, 0, 1},
			{-3, 3, 0, 0},
			{0, 0, -3, 3},
		});
		storing_matrix::Cubic_Hermite = Matrix::identity(4);
		storing_matrix::CatmullRom = Matrix({
			{0, 1, 0, 0},
			{0, 0, 1, 0},
			{-0.5, 0, 0.5, 0},
			{0, -0.5, 0, 0.5},
		});
		storing_matrix::Cubic_B_Spline = Matrix({
			{1, 4, 1, 0,},
			{0, 1, 4, 1,},
			{-3, 0, 3, 0,},
			{0, -3, 0, 3},
		}) * (1.0 / 6.0);
	}
}


// ---------- Class ----------

CubicSplineSegment::CubicSplineSegment(
	SplineType splineType, std::vector<std::vector<double>> points,
	double knot_parameter_alpha
) {
	// Initializes matrices (prevents accessing them when uninitialized)
	setMatrices();

	setSplineType(splineType);
	setPoints(points);

	this->knot_parameter_alpha = knot_parameter_alpha;
}

CubicSplineSegment::CubicSplineSegment()
	: CubicSplineSegment(SplineType::Cubic_Bezier, std::vector<std::vector<double>>(4, std::vector<double>(2))) {}

void CubicSplineSegment::setSplineType(SplineType splineType) {
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

SplineType CubicSplineSegment::getSplineType() {
	return splineType;
}

std::vector<std::vector<double>> CubicSplineSegment::getControlPoints() {
	return control_points;
}

Matrix &CubicSplineSegment::getCharacteristicMatrix() {
	return characteristic_matrix::Cubic_Hermite;
}

Matrix &CubicSplineSegment::getStoringMatrix() {
	switch (splineType) {
		case Cubic_Bezier:
			return storing_matrix::Cubic_Bezier;
		case Cubic_Hermite:
			return storing_matrix::Cubic_Hermite;
		case CatmullRom:
			return storing_matrix::CatmullRom;
		case Cubic_B_Spline:
			return storing_matrix::Cubic_B_Spline;
		default:
			break;
	}
	return storing_matrix::Cubic_Bezier;
}

/*
void CubicSplineSegment::setScalingMatrix(double knot_parameter_alpha) {
	this->knot_parameter_alpha = knot_parameter_alpha;

	if (splineType != CatmullRom) {
		scalingMatrix = Matrix::identity(4);
		return;
	}

	std::vector<double> t_arr(4, 0);
	for (int i = 1; i < 4; i++) {
		t_arr[i] = t_arr[i - 1] + std::pow(
			aespa_lib::genutil::euclideanDistance(control_points[i - 1], control_points[i]),
			knot_parameter_alpha
		);
	}
	scalingMatrix = Matrix({
		{1, 0, 0, 0},
		{0, 1, 0, 0},
		{0, 0, 1.0 / (t_arr[2] - t_arr[0]), 0},
		{0, 0, 0, 1.0 / (t_arr[3] - t_arr[1])}
	});
}
	*/

std::vector<double> CubicSplineSegment::getPositionAtT(double t) {
	Matrix t_matrix({ {1, t, t*t, t*t*t} });
	Matrix point_matrix = Matrix(stored_points);
	std::vector<double> point = t_matrix.multiply(getCharacteristicMatrix()).multiply(point_matrix).data[0];
	return point;
}

std::vector<double> CubicSplineSegment::getFirstPrimeAtT(double t) {
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

std::shared_ptr<SegmentBase> CubicSplineSegment::getReversed() {
	// Create new segment of same type
	CubicSplineSegment resultSegment;
	resultSegment.setSplineType(splineType);

	// Set reversed control points
	std::vector<std::vector<double>> newControlPoints = control_points;
	std::reverse(newControlPoints.begin(), newControlPoints.end());
	resultSegment.setPoints(newControlPoints);

	// Return segment
	return std::shared_ptr<SegmentBase>(new CubicSplineSegment(resultSegment));
}


}
}
}
