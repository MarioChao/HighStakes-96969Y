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
	double dot(Vector2D other);

	double x, y;
};


}
}
