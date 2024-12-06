#pragma once

namespace redirect {
	void runThread();

	void preauton();

	int getState();
	void setState(int, double = 0);

	void switchState();

	void control(int);

	bool canControl();

	extern int _taskState;
	extern double _taskDelay;
}  // namespace redirect
