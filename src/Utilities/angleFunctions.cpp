#include "Utilities/angleFunctions.h"
#include "main.h"

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

	double sinc(double x) {
		x = fabs(x) + 1e-14;
		return sin(x) / x;
	}
}
