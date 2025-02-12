#pragma once

namespace botintakelift {
	void setState(int, double = 0);

	void switchState();

	bool canControl();

	extern int _taskState;
	extern double _taskDelay;
}
