#include "Mechanics/goalClamp.h"
#include "main.h"

namespace {
    bool controlState = true;
}

namespace goalclamp {
	void startThread() {
		while (true) {
			// Thread code here

			task::sleep(20);
		}
	}

	void preauton() {
		GoalClampPneumatic.set(0);
	}

	void setState(int state, double delaySec) {
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