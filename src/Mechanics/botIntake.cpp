#include "Mechanics/botIntake.h"
#include "main.h"

namespace {
    void resolveIntake();

	double intakeVelocityPct = 80;

    int resolveState = 0;

    bool controlState = true;
}

namespace botintake {
	void startThread() {
		while (true) {
			resolveIntake();
			task::sleep(20);
		}
	}

	void preauton() {

	}

	void setState(int state, double delaySec) {
		// Set global variables
		_taskState = state;
		_taskDelay = delaySec;

		task setState([] () -> int {
			// Get global variables
			int taskState = _taskState;
			double taskDelay = _taskDelay;

			// Delay setting state
			if (taskDelay > 1e-9) {
				task::sleep(taskDelay * 1000);
			}

			// Set state here
			resolveState = taskState;

			return 1;
		});
	}

	void control() {
		if (canControl()) {
			int intakeDirection = (int) Controller1.ButtonR1.pressing() - (int) Controller1.ButtonR2.pressing();
			setState(-intakeDirection);
		}
	}

	bool canControl() {
		return controlState;
	}

	int _taskState;
	double _taskDelay;
}

namespace {
    /// @brief Set the intake to Holding (0) or Released (1). Intake state is modified by setIntakeResolveState(int).
    void resolveIntake() {
        // Make sure intakeResolveState is within [-1, 1]
        resolveState = (resolveState > 0) - (resolveState < 0);

        // Resolve intake
        if (resolveState == 1) {
            // Forward
            IntakeMotors.spin(fwd, intakeVelocityPct, pct);
        } else if (resolveState == -1) {
            // Reversed
            IntakeMotors.spin(fwd, -intakeVelocityPct, pct);
        } else {
            // Hold
            IntakeMotors.stop(brakeType::coast);
		}
    }
}