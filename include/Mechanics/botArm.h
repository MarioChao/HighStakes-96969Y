#pragma once

namespace botarm {
	void runThread();

	void preauton();

	void setTargetAngle(double, double = 0);
	double getTargetAngle();

	void setArmStage(int stageId, double delay_sec = 0, double maxSpeed_pct = 100);
	int getArmStage();
	void resetArmEncoder();
	bool isArmResetted();
	void setResetDefaultStage(int stageId);

	void switchState();

	void control(int);

	bool canControl();

	extern double _taskState;
	extern double _taskDelay;
}
