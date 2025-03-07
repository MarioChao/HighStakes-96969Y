#include "Pas1-Lib/Auton/End-Conditions/settle.h"

namespace pas1_lib {
namespace auton {
namespace end_conditions {

Settle::Settle(double errorMargin, double timeout_seconds)
	: errorMargin(errorMargin), timeout_seconds(timeout_seconds) {
	reset();
}

void Settle::reset() {
	settleTimer.reset();
	isSettled_state = false;
}

void Settle::computeFromError(double error) {
	if (fabs(error) > errorMargin) {
		reset();
	}
}

void Settle::settleNow() {
	isSettled_state = true;
}

bool Settle::isSettled() {
	if (isSettled_state) {
		return true;
	}

	if (settleTimer.time(seconds) >= timeout_seconds) {
		isSettled_state = true;
	}
	return isSettled_state;
}

}
}
}
