#include "Autonomous/autonFunctions.h"

#include "main.h"

namespace {
	double setFrontWings_DelaySec;
	double setLeftWing_DelaySec;
	double setRightWing_DelaySec;
	double setWings_DelaySec;
	bool setFrontWings_WingState;
	bool setLeftWing_LeftWingState;
	bool setRightWing_RightWingState;
	bool setWings_WingsState;
}

namespace autonfunctions {
	/// @brief Set the state of Front Wings's pneumatic.
	/// @param state Expanded: true, retracted: false.
	/// @param delaySec Number of seconds to wait before setting the pneumatic state (in a task).
	void setFrontWingsState(bool state, double delaySec) {
		setFrontWings_WingState = state;
		setFrontWings_DelaySec = delaySec;
		task setPneumaticState([]() -> int {
			int taskState = setFrontWings_WingState;

			if (setFrontWings_DelaySec > 1e-9) {
				task::sleep(setFrontWings_DelaySec * 1000);
			}
			FrontWingsPneumatic.set(taskState);
			return 1;
		});
	}

	/// @brief Set the state of Left Wing's pneumatic.
	/// @param state Expanded: true, retracted: false.
	/// @param delaySec Number of seconds to wait before setting the pneumatic state (in a task).
	void setLeftWingState(bool state, double delaySec) {
		setLeftWing_LeftWingState = state;
		setLeftWing_DelaySec = delaySec;
		task setPneumaticState([]() -> int {
			int taskState = setLeftWing_LeftWingState;

			if (setLeftWing_DelaySec > 1e-9) {
				task::sleep(setLeftWing_DelaySec * 1000);
			}
			LeftWingPneumatic.set(taskState);
			return 1;
		});
	}

	/// @brief Set the state of Right Wing's pneumatic.
	/// @param state Expanded: true, retracted: false.
	/// @param delaySec Number of seconds to wait before setting the pneumatic state (in a task).
	void setRightWingState(bool state, double delaySec) {
		setRightWing_RightWingState = state;
		setRightWing_DelaySec = delaySec;
		task setPneumaticState([]() -> int {
			int taskState = setRightWing_RightWingState;

			if (setRightWing_DelaySec > 1e-9) {
				task::sleep(setRightWing_DelaySec * 1000);
			}
			RightWingPneumatic.set(taskState);
			return 1;
		});
	}

	/// @brief Set the state of Left and Right Wing's pneumatic.
	/// @param state Expanded: true, retracted: false.
	/// @param delaySec Number of seconds to wait before setting the pneumatic state (in a task).
	void setBackWingsState(bool state, double delaySec) {
		setWings_WingsState = state;
		setWings_DelaySec = delaySec;
		task setPneumaticsState([]() -> int {
			int taskState = setWings_WingsState;

			if (setWings_DelaySec > 1e-9) {
				task::sleep(setWings_DelaySec * 1000);
			}
			LeftWingPneumatic.set(taskState);
			RightWingPneumatic.set(taskState);
			return 1;
		});
	}
}
