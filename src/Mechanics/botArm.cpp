#include "AutonUtilities/pidControl.h"
#include "Mechanics/botArm.h"
#include "Utilities/generalUtility.h"
#include "main.h"

namespace {
	void resolveArmDegrees();
	void resolveArmDirection();

	void setArmPosition(double position_degrees);
	void spinArmMotor(double velocityPct);

	PIDControl armPositionPid(1.0, 0.0, 0.1);

	std::vector<double> armStages_degrees = {-10, 195.0, 360.0};
	int currentArmStage = 0;

	double armVelocityPct = 100;
	double armUpVelocityPct = 100;

	double armStateTargetAngle_degrees = 0;
	int armStateDirection = 0;

	bool useDirection = false;

	bool controlState = true;
}

namespace botarm {
	void runThread() {
		while (true) {
			// Thread code here
			if (useDirection) {
				resolveArmDirection();
			} else {
				resolveArmDegrees();
			}

			task::sleep(20);
		}
	}

	void preauton() {
		ArmMotor.setPosition(0, degrees);
	}

	void setState(double state, double delaySec) {
		// Check for instant set
		if (delaySec <= 1e-9) {
			// Set state here
			armPositionPid.setErrorI(0);
			armStateTargetAngle_degrees = state;

			return;
		}

		// Set global variables
		_taskState = state;
		_taskDelay = delaySec;

		task setState([]() -> int {
			// Get global variables
			int taskState = _taskState;
			double taskDelay = _taskDelay;

			// Delay setting state
			task::sleep(taskDelay * 1000);

			// Set state here
			armPositionPid.setErrorI(0);
			armStateTargetAngle_degrees = taskState;

			return 1;
		});
	}

	void setArmStage(int stageId, double delaySec) {
		if (!(0 <= stageId && stageId < (int) armStages_degrees.size())) {
			return;
		}
		currentArmStage = stageId;
		setState(armStages_degrees[stageId], delaySec);
	}

	int getArmStage() {
		return currentArmStage;
	}

	void resetArmEncoder() {
		// Spin downward for 1 second
		currentArmStage = 100;
		armStateTargetAngle_degrees = -1000;
		task::sleep(1000);

		// Set the position as stage 0
		setArmPosition(0);
		setArmStage(0);
	}

	void control(int state) {
		if (canControl()) {
			armStateDirection = state;
			useDirection = true;
		}
	}

	bool canControl() {
		return controlState;
	}

	double _taskState;
	double _taskDelay;
}

namespace {
	void resolveArmDegrees() {
		// Calculate error
		double currentPosition_degrees = ArmRotationSensor.position(degrees);
		double error_degrees = armStateTargetAngle_degrees - currentPosition_degrees;
		printf("Err: %.3f\n", error_degrees);

		// Get pid value
		armPositionPid.computeFromError(error_degrees);
		// double motorDeltaVelocityPct = armPositionPid.getValue();
		double motorVelocityPct = armPositionPid.getValue();

		// Get final value
		// double motorVelocityPct = ArmMotor.velocity(pct) + motorDeltaVelocityPct;
		printf("PIDVal: %.3f\n", motorVelocityPct);
		motorVelocityPct = genutil::clamp(motorVelocityPct, -armVelocityPct, armVelocityPct);
		printf("MotVal: %.3f\n", motorVelocityPct);

		// Set velocity
		spinArmMotor(motorVelocityPct);
	}

	void resolveArmDirection() {
		armStateDirection = (armStateDirection > 0) - (armStateDirection < 0);

		switch (armStateDirection) {
			case 1:
				if (ArmMotor.position(deg) > 1000.0) {
					ArmMotor.stop(hold);
				} else {
					ArmMotor.spin(forward, genutil::pctToVolt(armUpVelocityPct), volt);
				}
				break;

			case -1:
				if (ArmMotor.position(deg) < 10.0) {
					ArmMotor.stop();
				} else {
					ArmMotor.spin(forward, -genutil::pctToVolt(armVelocityPct), volt);
				}
				break;

			default:
				ArmMotor.stop(hold);
				break;
		}
	}

	void setArmPosition(double position_degrees) {
		ArmRotationSensor.setPosition(position_degrees, degrees);
		armPositionPid.resetErrorToZero();
	}

	void spinArmMotor(double velocityPct) {
		// Spin
		double velocityVolt = genutil::pctToVolt(velocityPct);
		velocityVolt = genutil::clamp(velocityVolt, -10, 10);
		ArmMotor.spin(forward, velocityVolt, volt);
		// ArmMotor.spin(forward, velocityPct, pct);
	}
}
