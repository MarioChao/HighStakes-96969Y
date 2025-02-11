#pragma once

namespace botlift {
	void setState(int, double = 0);

	void switchState();

	bool canControl();

	extern int _taskState;
	extern double _taskDelay;
}
