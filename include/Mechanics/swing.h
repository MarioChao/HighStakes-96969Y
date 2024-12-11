#pragma once

namespace swing {
	void runThread();

	void preauton();

	void setState(int, double = 0);
	void set2ndState(int, double = 0);

	void switchState();
	void switch2ndState();

	void control(int);

	bool canControl();

	extern int _taskState;
	extern double _taskDelay;
}
