#pragma once

namespace angle {
	/// @brief Return the modulo of a number within a range.
	/// @param num The dividend of the modulo.
	/// @param mod The divisor of the modulo.
	/// @param min The smallest possible number of the modulo range.
	/// @return num % mod in the range [min, min + mod).
	double modRange(double num, double mod, double min);
}
