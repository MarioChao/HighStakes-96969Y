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

	void reset(bool ignoreFirst = true);

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
