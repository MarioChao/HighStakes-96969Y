#include "Mechanics/goalClamp.h"
#include "main.h"

namespace {
	bool controlState = true;
}

namespace goalclamp {
	void runThread() {
		while (true) {
			// Thread code here

			task::sleep(20);
		}
	}

	void preauton() {
		GoalClampPneumatic.set(0);
	}

	void setState(int state, double delaySec) {
		// Check for instant set
		if (delaySec <= 1e-9) {
			// Set state here
			GoalClampPneumatic.set(state);

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
			GoalClampPneumatic.set(taskState);

			return 1;
		});
	}

	void switchState() {
		setState(!GoalClampPneumatic.value());
	}

	void control() {
		if (canControl()) {
			// Control code here
		}
	}

	bool canControl() {
		return controlState;
	}

	int _taskState;
	double _taskDelay;
}

namespace {
}
