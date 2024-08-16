#include "Mechanics/botHang.h"
#include "main.h"

namespace bothang {
    void setState(bool state, double delaySec) {
		// Set global variables
		_taskState = state;
		_taskDelay = delaySec;

		task setState([] () -> int {
			// Get global variables
			int taskState = _taskState;
			double taskDelay = _taskDelay;

			// Delay setting state
			if (taskDelay > 1e-9) {
				task::sleep(taskDelay * 1000);
			}

			// Set state here
			printf("New state: %d\n", taskState);
			HangPneumatic.set(taskState);

			return 1;
		});
	}

	void switchState() {
		setState(!HangPneumatic.value());
	}
}
