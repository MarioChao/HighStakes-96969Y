#include "Mechanics/swing.h"
#include "main.h"

namespace {
	bool controlState = true;
}

namespace swing {
	void runThread() {
		while (true) {
			// Thread code here

			task::sleep(20);
		}
	}

	void preauton() {

	}

	void setState(int state, double delaySec) {
		// Check for instant set
		if (delaySec <= 1e-9) {
			// Set state here
			SwordPneumatics.set(state);
			printf("Sword 1 state: %d\n", SwordPneumatics.value());

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
			SwordPneumatics.set(taskState);

			return 1;
		});
	}

	void set2ndState(int state, double delaySec) {
		// Check for instant set
		if (delaySec <= 1e-9) {
			// Set state here
			Sword2Pneumatics.set(state);

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
			Sword2Pneumatics.set(taskState);

			return 1;
		});
	}

	void switchState() {
		setState(!SwordPneumatics.value());
	}

	void switch2ndState() {
		set2ndState(!Sword2Pneumatics.value());
	}

	void control(int state) {
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
