#pragma once

namespace botintake {
	void runThread();
	
	void preauton();
	
	void setState(int, double = 0);
	
	void control();
	
	bool canControl();

	extern int _taskState;
	extern double _taskDelay;
}