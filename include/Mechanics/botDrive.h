#pragma once

namespace botdrive {
	enum controlType {
		ArcadeTwoStick,
		ArcadeSingleStick,
	};

	void preauton();

	void switchDriveMode();

	void control();

	void setMaxDriveVelocity(double velocityPct);
	double getMaxDriveVelocity();

	void driveVelocity(double leftVelocityPct, double rightVelocityPct, bool useCustomPid = false);
	void driveVoltage(double leftVoltageVolt, double rightVoltageVolt, double clampMaxVoltage);
}
