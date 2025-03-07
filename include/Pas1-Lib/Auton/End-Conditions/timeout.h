#pragma once

#include "vex.h"

namespace pas1_lib {
namespace auton {
namespace end_conditions {

class Timeout {
public:
	Timeout(double timeout_seconds);

	void reset();

	bool isExpired();

private:
	timer timeoutTimer;

	double timeout_seconds;
};

}
}
}
