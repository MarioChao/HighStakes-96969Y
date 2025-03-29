#pragma once

namespace climb_pto {
	void runThread();

	void preauton();

	void setState(int state, double delaySec = 0);

	void switchState();

	void control(int);

	bool canControl();

	extern int _taskState;
	extern double _taskDelay;
}
