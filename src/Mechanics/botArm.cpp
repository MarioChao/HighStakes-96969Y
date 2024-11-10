#include "AutonUtilities/pidControl.h"
#include "Mechanics/botArm.h"
#include "Utilities/generalUtility.h"
#include "main.h"

namespace {
	void resolveArmDegrees();
	void resolveArmDirection();

	PIDControl armMaintainPositionPid(0.4, 0.01, 0.0);

	double armVelocityPct = 90;
	double armUpVelocityPct = 70;

	double armVelocityVolt = armVelocityPct / 100.0 * 12.0;
	double armUpVelocityVolt = armUpVelocityPct / 100.0 * 12.0;

	int armStateDegrees = 0;
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

	void setState(int state, double delaySec) {
		// Check for instant set
		if (delaySec <= 1e-9) {
			// Set state here
			armMaintainPositionPid.setErrorI(0);
			armStateDegrees = state;

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
			armMaintainPositionPid.setErrorI(0);
			armStateDegrees = taskState;

			return 1;
		});
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

	int _taskState;
	double _taskDelay;
}

namespace {
	void resolveArmDegrees() {
		// Calculate error
		double degreesError = armStateDegrees - ArmMotor.position(degrees);

		// Get pid value
		armMaintainPositionPid.computeFromError(degreesError);
		double motorDeltaVelocityPct = armMaintainPositionPid.getValue();

		// Get final value
		double motorVelocityPct = ArmMotor.velocity(pct) + motorDeltaVelocityPct;
		motorVelocityPct = genutil::clamp(motorVelocityPct, -armVelocityPct, armVelocityPct);

		// Set velocity
		ArmMotor.spin(forward, motorVelocityPct / 100.0 * 12.0, volt);
	}

	void resolveArmDirection() {
		armStateDirection = (armStateDirection > 0) - (armStateDirection < 0);

		switch (armStateDirection) {
			case 1:
				if (ArmMotor.position(deg) > 340.0) {
					ArmMotor.stop(hold);
				} else {
					ArmMotor.spin(forward, armUpVelocityPct, volt);
				}
				break;

			case -1:
				if (ArmMotor.position(deg) < 10.0) {
					ArmMotor.stop();
				} else {
					ArmMotor.spin(forward, -armVelocityVolt, volt);
				}
				break;

			default:
				ArmMotor.stop(hold);
				break;
		}
	}
}
