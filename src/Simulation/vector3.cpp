#include "Simulation/vector3.h"

#include <cmath>

Vector3::Vector3(double x, double y, double z) {
	this->x = x;
	this->y = y;
	this->z = z;
}

Vector3 Vector3::operator+(const Vector3 &other) {
	Vector3 newVector(*this);
	newVector += other;
	return newVector;
}

Vector3 &Vector3::operator+=(const Vector3 &other) {
	x += other.x;
	y += other.y;
	z += other.z;
	return *this;
}

Vector3 Vector3::operator-(const Vector3 &other) {
	Vector3 newVector(*this);
	newVector -= other;
	return newVector;
}

Vector3 &Vector3::operator-=(const Vector3 &other) {
	x -= other.x;
	y -= other.y;
	z -= other.z;
	return *this;
}

Vector3 Vector3::operator*(double s) {
	Vector3 newVector(*this);
	newVector *= s;
	return newVector;
}

Vector3 &Vector3::operator*=(double s) {
	x *= s;
	y *= s;
	z *= s;
	return *this;
}

double Vector3::getMagnitude() {
	return sqrt(x * x + y * y + z * z);
}
