#include "GraphUtilities/cubicSpline.h"

CubicSpline::CubicSpline()
: character_matrix(cspline::bezier_character_matrix),
control_matrices(std::pair<Matrix, Matrix>(Matrix(4, 1), Matrix(4, 1)))
{}

CubicSpline::CubicSpline(Matrix characteristic, Matrix x, Matrix y)
: character_matrix(characteristic),
control_matrices(std::pair<Matrix, Matrix>(x, y))
{}

void CubicSpline::setCharacteristicMatrix(Matrix matrix) {
	character_matrix = matrix;
}

void CubicSpline::setControlMatrices(Matrix matrix_x, Matrix matrix_y) {
	control_matrices = std::make_pair(matrix_x, matrix_y);
}

std::pair<double, double> CubicSpline::getPositionAtT(double t) {
	Matrix t_matrix({{1, t, t*t, t*t*t}});
	Matrix first_matrix = t_matrix.multiply(character_matrix);
	double x = first_matrix.multiply(control_matrices.first).data[0][0];
	double y = first_matrix.multiply(control_matrices.second).data[0][0];
	return std::make_pair(x, y);
}

std::pair<double, double> CubicSpline::getVelocityAtT(double t) {
	Matrix t_matrix({{0, 1, 2*t, 3*t*t}});
	Matrix first_matrix = t_matrix.multiply(character_matrix);
	double dx = first_matrix.multiply(control_matrices.first).data[0][0];
	double dy = first_matrix.multiply(control_matrices.second).data[0][0];
	return std::make_pair(dx, dy);
}

namespace cspline {
	Matrix bezier_character_matrix({
		{1, 0, 0, 0},
		{-3, 3, 0, 0},
		{3, -6, 3, 0},
		{-1, 3, -3, 1},
	});
	Matrix hermite_character_matrix({
		{1, 0, 0, 0},
		{0, 1, 0, 0},
		{-3, -2, 3, -1},
		{2, 1, -2, 1},
	});
	Matrix catmull_rom_character_matrix = Matrix({
		{0, 2, 0, 0},
		{-1, 0, 1, 0},
		{2, -5, 4, -1},
		{-1, 3, -3, 1},
	}) * 0.5;
	Matrix b_spline_character_matrix = Matrix({
		{1, 4, 1, 0},
		{-3, 0, 3, 0},
		{3, -6, 3, 0},
		{-1, 3, -3, 1},
	}) * (1.0 / 6.0);
}
