#include "Autonomous/autonPaths.h"


namespace {
bool doAllianceStake = true;
}


namespace autonpaths {
namespace configs {
bool willDoAllianceStake() {
	return doAllianceStake;
}

void setDoAllianceStake(bool state) {
	doAllianceStake = state;
}
}
}
