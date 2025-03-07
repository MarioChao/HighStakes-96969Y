#include "Pas1-Lib/Auton/End-Conditions/timeout.h"

namespace pas1_lib {
namespace auton {
namespace end_conditions {

Timeout::Timeout(double timeout_seconds)
	: timeout_seconds(timeout_seconds) {
	reset();
}

void Timeout::reset() {
	timeoutTimer.reset();
}

bool Timeout::isExpired() {
	return timeoutTimer.time(seconds) >= timeout_seconds;
}

}
}
}
