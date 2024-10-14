#pragma once

namespace botlift {
	void setState(bool, double = 0);

	void switchState();

	extern int _taskState;
	extern double _taskDelay;
}
