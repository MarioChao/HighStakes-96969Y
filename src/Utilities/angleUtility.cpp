#include "Utilities/angleUtility.h"

#include <cmath>

namespace angle {
	double modRange(double num, double mod, double min) {
		// Offset from minimum
		double ret = fmod(num - min, mod);
		// Get positive
		if (ret < 0) ret += fabs(mod);
		// Offset to minimum
		ret += min;
		return ret;
	}

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
