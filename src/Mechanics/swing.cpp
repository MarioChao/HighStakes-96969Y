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

void setState_left(int state, double delaySec) {
	// Check for instant set
	if (delaySec <= 1e-9) {
		// Set state here
		LeftSword_pneumatics.set(state);

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
		LeftSword_pneumatics.set(taskState);

		return 1;
	});
}

void set2ndState_left(int state, double delaySec) {
	// Check for instant set
	if (delaySec <= 1e-9) {
		// Set state here
		LeftSword2_pneumatics.set(state);

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
		LeftSword2_pneumatics.set(taskState);

		return 1;
	});
}

void setState_right(int state, double delaySec) {
	// Check for instant set
	if (delaySec <= 1e-9) {
		// Set state here
		RightSword_pneumatics.set(state);
		return;
	}

	// Set global variables
	_taskState = state;
	_taskDelay = delaySec;

	task setState([]() -> int {
		int taskState = _taskState;
		double taskDelay = _taskDelay;
		task::sleep(taskDelay * 1000);
		// Set state here
		RightSword_pneumatics.set(taskState);
		return 1;
	});
}

void set2ndState_right(int state, double delaySec) {
	// Check for instant set
	if (delaySec <= 1e-9) {
		// Set state here
		RightSword2_pneumatics.set(state);
		return;
	}

	// Set global variables
	_taskState = state;
	_taskDelay = delaySec;

	task setState([]() -> int {
		int taskState = _taskState;
		double taskDelay = _taskDelay;
		task::sleep(taskDelay * 1000);
		// Set state here
		RightSword2_pneumatics.set(taskState);
		return 1;
	});
}

void switchState_left() { setState_left(!LeftSword_pneumatics.value()); }
void switch2ndState_left() { set2ndState_left(!LeftSword2_pneumatics.value()); }
void switchState_right() { setState_right(!RightSword_pneumatics.value()); }
void switch2ndState_right() { set2ndState_right(!RightSword2_pneumatics.value()); }

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
