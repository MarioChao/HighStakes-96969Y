#include "Aespa-Lib/Karina-Data-Structures/matrix.h"

#include <cstdio>

namespace aespa_lib {
namespace datas {

Matrix::Matrix(int d1, int d2) {
	data = std::vector< std::vector<double> >(
		d1, std::vector<double>(d2, 0)
	);
	shape = std::make_pair(d1, d2);
}

Matrix::Matrix(std::vector<std::vector<double>> matrix_data) {
	// Get maximum column
	int d1 = (int) matrix_data.size();
	int d2 = 0;
	for (int row = 0; row < d1; row++) {
		int tmp_d2 = (int) matrix_data[row].size();
		d2 = std::max(tmp_d2, d2);
	}

	// Resize matrix to d1 x d2
	data = matrix_data;
	for (int row = 0; row < d1; row++) {
		data[row].resize(d2, 0);
	}
	shape = std::make_pair(d1, d2);
}

Matrix Matrix::multiply(Matrix &other) {
	// Validate possible
	if (!canMultiply(other)) {
		printf("Can't multiply!\n");
		return Matrix(0, 0);
	}

	// Create result shape
	int d1 = shape.first;
	int d2 = other.shape.second;
	Matrix result(d1, d2);

	// Multiply matrices
	int d_mid = shape.second;
	for (int row = 0; row < d1; row++) {
		for (int clm = 0; clm < d2; clm++) {
			for (int i = 0; i < d_mid; i++) {
				double value = data[row][i] * other.data[i][clm];
				result.data[row][clm] += value;
			}
		}
	}

	// Return result
	return result;
}

bool Matrix::canMultiply(Matrix &other) {
	// Check if (matrix 1's dim 2) == (matrix 2's dim 1)
	return shape.second == other.shape.first;
}

Matrix Matrix::operator*(double s) {
	Matrix newMatrix(*this);
	newMatrix *= s;
	return newMatrix;
}

Matrix &Matrix::operator*=(double s) {
	for (int row = 0; row < shape.first; row++) {
		for (int clm = 0; clm < shape.second; clm++) {
			data[row][clm] *= s;
		}
	}
	return *this;
}

std::pair<int, int> Matrix::getShape() {
	return shape;
}

std::string Matrix::getString() {
	std::string result = "";
	for (int row = 0; row < shape.first; row++) {
		for (int clm = 0; clm < shape.second; clm++) {
			char num[21];
			snprintf(num, 20, "%.3f\t", data[row][clm]);
			result += num;
		}
		result += "\n";
	}
	return result;
}

Matrix Matrix::identity(int dim) {
	Matrix result(dim, dim);
	for (int i = 0; i < dim; i++) {
		result.data[i][i] = 1;
	}
	return result;
}

}
}
