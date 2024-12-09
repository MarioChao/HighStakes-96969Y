#pragma once

#include <string>

namespace botintake2 {
	void runThread();

	void preauton();

	void setState(int, double = 0);

	void setState2(int, double = 0);

	void setState3(int, double = 0);

	void switchMode();

	void setHookMode(int);

	void switchFilterColor();

	void setFilterOutColor(std::string);

	void control(int, int);

	bool canControl();

	extern int _taskState;
	extern double _taskDelay;
}
