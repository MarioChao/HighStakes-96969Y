#pragma once

namespace botdrive {
	enum controlType {
		ArcadeTwoStick,
		ArcadeSingleStick,
	};

	void runThread();

	void preauton();

	void switchDriveMode();

	void control();

	bool canControl();
	void setControlState(bool canControl);

	void setMaxDriveVelocity(double velocityPct);
	double getMaxDriveVelocity();
}
