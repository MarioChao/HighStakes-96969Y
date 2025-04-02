#include "Aespa-Lib/Karina-Data-Structures/linegular.h"

#include <cmath>
#include "Aespa-Lib/Winter-Utilities/angle.h"
#include "Aespa-Lib/Winter-Utilities/general.h"


namespace aespa_lib {
namespace datas {


// ---------- Vector2D ----------

Vector2D::Vector2D(double x, double y)
	: x(x), y(y) {}

void Vector2D::rotateBy(units::PolarAngle rotation) {
	double radians = rotation.polarRad();
	double newX = x * cos(radians) - y * sin(radians);
	double newY = x * sin(radians) + y * cos(radians);
	x = newX;
	y = newY;
}

void Vector2D::rotateExponentialBy(units::PolarAngle rotation) {
	// Check pose exponential in https://file.tavsys.net/control/controls-engineering-in-frc.pdf
	double radians = rotation.polarRad();
	double newX = x * aespa_lib::angle::sinc(radians) + y * aespa_lib::angle::cosm1_x(radians);
	double newY = x * -aespa_lib::angle::cosm1_x(radians) + y * aespa_lib::angle::sinc(radians);
	x = newX;
	y = newY;
}

double Vector2D::getMagnitude() {
	return aespa_lib::genutil::l2Norm({ x, y });
}


// ---------- Linegular ----------

Linegular::Linegular(Vector2D position, units::PolarAngle rotation)
	: position(position),
	rotation(rotation) {}

Linegular::Linegular(double x, double y, units::PolarAngle rotation)
	: Linegular(Vector2D(x, y), rotation) {}

Vector2D Linegular::getPosition() {
	return position;
}

double Linegular::getX() {
	return position.x;
}

double Linegular::getY() {
	return position.y;
}

units::PolarAngle Linegular::getRotation() {
	return rotation;
}

double Linegular::getXYMagnitude() {
	return position.getMagnitude();
}

void Linegular::setPosition(Vector2D newPosition) {
	position = newPosition;
}

void Linegular::setPosition(double x, double y) {
	setPosition(Vector2D(x, y));
}

void Linegular::setRotation(units::PolarAngle newRotation) {
	rotation = newRotation;
}

void Linegular::rotateXYBy(units::PolarAngle rotation) {
	position.rotateBy(rotation);
}

void Linegular::rotateExponentialBy(units::PolarAngle rotation) {
	position.rotateExponentialBy(rotation);
}

Linegular Linegular::operator+(Linegular other) {
	return Linegular(getX() + other.getX(), getY() + other.getY(), rotation + other.rotation);
}

Linegular Linegular::operator-(Linegular other) {
	return Linegular(getX() - other.getX(), getY() - other.getY(), rotation - other.rotation);
}

Linegular &Linegular::operator+=(Linegular other) {
	position.x += other.getX();
	position.y += other.getY();
	rotation += other.rotation;
	return *this;
}

Linegular &Linegular::operator-=(Linegular other) {
	position.x -= other.getX();
	position.y -= other.getY();
	rotation -= other.rotation;
	return *this;
}

Linegular Linegular::operator*(double value) {
	return Linegular(getX() * value, getY() * value, rotation);
}

Linegular Linegular::operator/(double value) {
	return operator*(1 / value);
}

}
}
