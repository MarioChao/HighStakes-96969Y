#pragma once

#include "vex.h"


namespace pas1_lib {
namespace auton {
namespace control_loops {

class SlewController {
public:
	/// @brief Constructor
	/// @param maxChangeRate_valPerSec Use a negative value to disable the controller.
	SlewController(double maxChangeRate_valPerSec);
	SlewController();

	void reset(double defaultValue = 0, bool ignoreFirst = false);

	void computeFromTarget(double targetValue);

	double getValue();


	double maxChangeRate_valPerSec;

private:
	timer slewTimer;

	bool ignoreFirst;

	double storedValue;
};

}
}
}
