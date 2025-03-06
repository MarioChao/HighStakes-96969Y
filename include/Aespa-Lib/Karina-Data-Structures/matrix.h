#pragma once

#include <vector>
#include <string>

namespace aespa_lib {
namespace datas {

class Matrix {
public:
	Matrix(int d1, int d2);
	Matrix(std::vector<std::vector<double>> matrix_data);

	Matrix multiply(Matrix &other);
	bool canMultiply(Matrix &other);

	Matrix operator*(double s);
	Matrix &operator*=(double s);

	std::pair<int, int> getShape();

	std::string getString();

	std::vector<std::vector<double>> data;

	static Matrix identity(int dim);

private:
	std::pair<int, int> shape;
};

}
}
