#pragma once

#include "GraphUtilities/matrix.h"

#include <utility>

class CubicSpline {
public:
	CubicSpline();
	CubicSpline(Matrix characteristic, Matrix x, Matrix y);

	void setCharacteristicMatrix(Matrix matrix);
	void setControlMatrices(Matrix matrix_x, Matrix matrix_y);

	std::pair<double, double> getPositionAtT(double t);
	std::pair<double, double> getVelocityAtT(double t);

private:
	Matrix character_matrix;
	std::pair<Matrix, Matrix> control_matrices;
};

namespace cspline {
	extern Matrix bezier_character_matrix;
	extern Matrix hermite_character_matrix;
	extern Matrix catmull_rom_character_matrix;
	extern Matrix b_spline_character_matrix;
}
