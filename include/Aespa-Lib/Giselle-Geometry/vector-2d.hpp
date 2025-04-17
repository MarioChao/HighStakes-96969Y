#pragma once

#include "Aespa-Lib/Winter-Utilities/units.h"


namespace aespa_lib {
namespace geometry {


struct Vector2D {
	Vector2D(double x, double y);
	static Vector2D fromPolar(units::PolarAngle angle, double magnitude);

	void rotateBy(units::PolarAngle rotation);
	void rotateExponentialBy(units::PolarAngle rotation);

	double getMagnitude();
	Vector2D getNormalized();

	Vector2D operator-();
	Vector2D operator+(Vector2D other);
	Vector2D operator-(Vector2D other);

	double cross_scalar(Vector2D other);
	double dot(Vector2D other);
	units::PolarAngle angleFrom(Vector2D other);

	double x, y;
};


}
}
