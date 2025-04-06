#include "MatchSequence/match-end.h"

#include "Mechanics/botArm.h"
#include "Mechanics/goalClamp.h"
#include "Mechanics/swing.h"
#include "Mechanics/botIntakeLift.h"
#include "global-vars.h"
#include "main.h"


namespace {
	void drivingEndedThread();
}


namespace match_end {
	void startThread() {
		task drivingEnded([]() -> int {
			drivingEndedThread();
			return 1;
		});
	}
}


namespace {
	void drivingEndedThread() {
		waitUntil(drivingTimer.time(seconds) > 3);
		while (true) {
			waitUntil(Competition.isDriverControl() && Competition.isEnabled());
			wait(100, msec);
			waitUntil(!Competition.isEnabled() || drivingTimer.time(seconds) > 104);
			if (!Competition.isDriverControl()) {
				continue;
			}
			break;
		}
		
		botarm::setArmStage(0); // motor doesn't spin after disable

		printf("Ended\n");

		/* Post match control */
		wait(20, msec);
		swing::setState(0);
		goalclamp::setState(0);
		// botintakelift::setState(1);
	}
}
