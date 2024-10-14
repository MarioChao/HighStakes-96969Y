#include "Mechanics/botLift.h"
#include "main.h"

namespace botlift {
	void setLiftState(bool state, double delaySec) {
		// Check for instant set
		if (delaySec <= 1e-9) {
			// Set state here
			IntakeLiftPneumatic.set(state);

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
			IntakeLiftPneumatic.set(taskState);

			return 1;
		});
	}

	void switchState() {
		// setState(!IntakeLiftPneumatic.value());
	}
}
