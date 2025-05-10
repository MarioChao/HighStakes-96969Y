#include "Autonomous/autonPaths.h"


namespace {
bool doAllianceStake = true;
bool touchLadder = false;
}


namespace autonpaths {
namespace configs {
bool willDoAllianceStake() {
	return doAllianceStake;
}

void setDoAllianceStake(bool state) {
	doAllianceStake = state;
}

bool willTouchLadder() {
	return touchLadder;
}

void setWillTouchLadder(bool state) {
	touchLadder = state;
}
}
}
