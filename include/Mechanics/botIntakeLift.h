#pragma once

namespace botintakelift {
	void setState(int state, double delaySec = 0);

	void switchState();

	bool canControl();

	extern int _taskState;
	extern double _taskDelay;
}
