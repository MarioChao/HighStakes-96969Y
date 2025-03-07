#pragma once

#include "vex.h"

namespace pas1_lib {
namespace auton {
namespace end_conditions {

class Settle {
public:
	Settle(double errorMargin, double timeout_seconds);

	void reset();
	void computeFromError(double error);

	void settleNow();
	bool isSettled();

private:
	timer settleTimer;

	double errorMargin;
	double timeout_seconds;

	bool isSettled_state;
};

}
}
}
