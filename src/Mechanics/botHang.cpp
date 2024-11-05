#include "Mechanics/botHang.h"
#include "main.h"

namespace bothang {
	void setState(bool state, double delaySec) {
		// Check for instant set
		if (delaySec <= 1e-9) {
			// Set state here
			HangPneumatic.set(state);

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
			HangPneumatic.set(taskState);

			return 1;
		});
	}

	void switchState() {
		setState(!HangPneumatic.value());
	}
}
