#pragma once

class Vector3 {
public:
	Vector3(double x = 0, double y = 0, double z = 0);

	Vector3 operator+(const Vector3 &other);
	Vector3 &operator+=(const Vector3 &other);

	Vector3 operator-(const Vector3 &other);
	Vector3 &operator-=(const Vector3 &other);

	Vector3 operator*(double s);
	Vector3 &operator*=(double s);

	double getMagnitude();

	double x, y, z;
};
