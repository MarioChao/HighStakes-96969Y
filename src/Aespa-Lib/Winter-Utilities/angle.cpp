#include "Aespa-Lib/Winter-Utilities/angle.h"

#include <cmath>

namespace aespa_lib {
namespace angle {

double swapFieldPolar_degrees(double degrees) {
	return 90 - degrees;
}

double sinc(double x) {
	// Approximate small x
	if (x < 1e-8) {
		// Taylor series approximation
		return 1 - pow(x, 2) / 6;
	}

	// Return expression
	return sin(x) / x;
}

double cosm1_x(double x) {
	// Approximate small x
	if (x < 1e-8) {
		// Taylor series approximation
		return -x / 2;
	}

	// Return expression
	return (cos(x) - 1) / x;
}

}
}
