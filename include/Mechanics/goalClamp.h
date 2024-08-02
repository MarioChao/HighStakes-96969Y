#pragma once

namespace goalclamp {
	void startThread();
	
	void preauton();
	
	void setState(int, double = 0);

	void switchState();
	
	void control();
	
	bool canControl();

	extern int _taskState;
	extern double _taskDelay;
}