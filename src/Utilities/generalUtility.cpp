#include "Utilities/generalUtility.h"
#include "main.h"

namespace genutil {
	double clamp(double value, double min, double max) {
		return fmin(max, fmax(min, value));
	}

	double pctToVolt(double pct) {
		return pct * 12.0 / 100.0;
	}

	int signum(double value) {
		if (value > 0) return 1;
		if (value == 0) return 0;
		return -1;
	}

	bool isWithin(double value, double target, double withinRange) {
		return fabs(value - target) <= withinRange;
	}
}
