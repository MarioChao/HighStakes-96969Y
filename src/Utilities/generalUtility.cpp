#include "Utilities/generalUtility.h"
#include "main.h"

namespace genutil {
	double clamp(double value, double min, double max) {
		return fmin(max, fmax(min, value));
	}
}
