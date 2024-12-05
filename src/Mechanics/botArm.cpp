#include "AutonUtilities/pidController.h"
#include "AutonUtilities/patienceController.h"
#include "Mechanics/botArm.h"
#include "Utilities/generalUtility.h"
#include "main.h"

namespace {
	void resolveArmExtreme();
	void resolveArmDegrees();
	void resolveArmDirection();

	bool isExtreme();

	void setArmPosition(double position_degrees);
	void spinArmMotor(double velocityPct);

	PIDController armPositionPid(1.0, 0, 0);
	PatienceController armUpPatience(50, 1.0, true);
	PatienceController armDownPatience(50, 1.0, false);

	std::vector<double> armStages_degrees = {0, 0, 220.0, 0};
	std::vector<int> extremeStages_values = {-1, -2, 0, 2};
	int currentArmStage = 0;
	bool releaseOnExhausted = true;

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
				printf("st: %d, armvolt: %.3f\n", currentArmStage, ArmMotor.voltage(volt));
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
		stageId = genutil::clamp(stageId, 0, (int) armStages_degrees.size() - 1);
		currentArmStage = stageId;
		
		// Extreme cases
		int stage_value = extremeStages_values[stageId];
		if (stage_value == -2) {
			// Down, hold
			armDownPatience.reset();
			releaseOnExhausted = false;
			setState(-1e7, delaySec);
			return;
		} else if (stage_value == -1) {
			// Down, release
			armDownPatience.reset();
			releaseOnExhausted = true;
			setState(-1e7, delaySec);
			return;
		} else if (stage_value == 1) {
			// Up, release
			armUpPatience.reset();
			releaseOnExhausted = true;
			setState(1e7, delaySec);
			return;
		} else if (stage_value == 2) {
			// Up, hold
			armUpPatience.reset();
			releaseOnExhausted = false;
			setState(1e7, delaySec);
			return;
		}

		// PID stage case
		setState(armStages_degrees[stageId], delaySec);
	}

	int getArmStage() {
		return currentArmStage;
	}

	void resetArmEncoder() {
		// Spin downward for 1 second
		// currentArmStage = 100;
		// armStateTargetAngle_degrees = -1000;
		// task::sleep(1000);

		// Spin downward until exhausted
		setArmStage(1);
		while (!armDownPatience.isExhausted()) {
			task::sleep(20);
		}

		// Set the position as 0 degrees
		setArmPosition(0);

		// Initialize to stage 0
		setArmStage(0);
		armDownPatience.computePatience(ArmRotationSensor.position(degrees));
		armDownPatience.exhaustNow();
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
	void resolveArmExtreme() {
		// Calculate error
		double currentPosition_degrees = ArmRotationSensor.position(degrees);
		double error_degrees = armStateTargetAngle_degrees - currentPosition_degrees;

		// Spin up or down
		if (error_degrees > 0) {
			/* Elevate */

			// Check patience
			armUpPatience.computePatience(currentPosition_degrees);
			if (armUpPatience.isExhausted()) {
				if (releaseOnExhausted) {
					ArmMotor.stop(coast);
				} else {
					spinArmMotor(3);
				}
				return;
			}

			// Spin
			spinArmMotor(armVelocityPct);
			
		} else if (error_degrees < 0) {
			/* Descend */

			// Check patience
			armDownPatience.computePatience(currentPosition_degrees);
			if (armDownPatience.isExhausted()) {
				if (releaseOnExhausted) {
					ArmMotor.stop(coast);
				} else {
					spinArmMotor(-3);
				}
				return;
			}

			// Spin
			spinArmMotor(-armVelocityPct);
		}
	}

	void resolveArmDegrees() {
		// Extreme case
		if (isExtreme()) {
			resolveArmExtreme();
			return;
		}

		// Calculate error
		double currentPosition_degrees = ArmRotationSensor.position(degrees);
		double error_degrees = armStateTargetAngle_degrees - currentPosition_degrees;
		// printf("Err: %.3f\n", error_degrees);

		// Get pid value
		armPositionPid.computeFromError(error_degrees);
		double motorVelocityPct = armPositionPid.getValue();

		// Get final value
		motorVelocityPct = genutil::clamp(motorVelocityPct, -armVelocityPct, armVelocityPct);
		// printf("MotVal: %.3f\n", motorVelocityPct);

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

	bool isExtreme() {
		if (!(0 <= currentArmStage && currentArmStage < (int) armStages_degrees.size())) {
			return false;
		}
		return extremeStages_values[currentArmStage] != 0;
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
