#include "Mechanics/botArmPneumatics.h"
#include "main.h"

namespace {
}

namespace botarmpneu {
	void setState(bool state, double delaySec) {
		// Check for instant set
		if (delaySec <= 1e-9) {
			// Set state here
			BotArmPneumatics.set(state);
			pressedCount++;

			return;
		}

		// Set global variables
		_taskState = state;
		_taskDelay = delaySec;

		task setState([]() -> int {
			// Get global variables
			int taskState = _taskState;
			double taskDelay = _taskDelay;

			// Delay setting state
			task::sleep(taskDelay * 1000);

			// Set state here
			BotArmPneumatics.set(taskState);
			pressedCount++;

			return 1;
		});
	}

	void switchState() {
		setState(!BotArmPneumatics.value());
	}

	int _taskState;
	double _taskDelay;

	double pressedCount = 0;
}

namespace {
}
