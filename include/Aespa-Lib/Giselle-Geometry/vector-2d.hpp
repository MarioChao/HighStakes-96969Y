#pragma once

#include "Aespa-Lib/Winter-Utilities/units.h"


namespace aespa_lib {
namespace geometry {


struct Vector2D {
	Vector2D(double x, double y);

	void rotateBy(units::PolarAngle rotation);
	void rotateExponentialBy(units::PolarAngle rotation);

	double getMagnitude();

	double x, y;
};


}
}
