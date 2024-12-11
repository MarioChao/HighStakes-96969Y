#include "Autonomous/autonFunctions.h"

#include "Mechanics/botIntake.h"
#include "Mechanics/botIntake2.h"
#include "Mechanics/botArm.h"
#include "Mechanics/botArmPneumatics.h"
#include "Mechanics/swing.h"
#include "Mechanics/goalClamp.h"
#include "Mechanics/redirect.h"

#include "main.h"

namespace {}

namespace autonfunctions {
	/// @brief Set the inertial sensor's absolute angle reading to a specified value. Doesn't turn the robot.
	/// @param rotation The angle (in degrees) to be set for the current orientation.
	void setRotation(double rotation) {
		InertialSensor.setRotation(rotation, deg);
	}

	timer _autonTimer;

	/// @brief Set the state of the intake.
	/// @param state Forward: 1, released: 0, reversed: -1
	/// @param delaySec Number of seconds to wait before setting the state (in a task).
	void setIntakeState(int state, double delaySec) {
		if (intakePart == 1) botintake::setState(state, delaySec);
		else botintake2::setState(state, delaySec);
	}


	/// @brief Set the state of the top intake.
	/// @param state Forward: 1, released: 0, reversed: -1
	/// @param delaySec Number of seconds to wait before setting the state (in a task).
	void setIntakeTopState(int state, double delaySec) {
		if (intakePart == 1) return;
		else botintake2::setState2(state, delaySec);
	}


	/// @brief Set the state of the bottom intake.
	/// @param state Forward: 1, released: 0, reversed: -1
	/// @param delaySec Number of seconds to wait before setting the state (in a task).
	void setIntakeBottomState(int state, double delaySec) {
		if (intakePart == 1) return;
		else botintake2::setState3(state, delaySec);
	}

	/// @brief Set the hook mode of the intake.
	/// @param state Normal: 0, to arm: 1
	void setIntakeToArm(int state) {
		if (intakePart == 1) {
			if (state) {
				botintake::setColorFiltering(false);
				redirect::setState(1);
				botarm::setArmStage(1);
			} else {
				botintake::setColorFiltering(true);
			}
		} else botintake2::setHookMode(state);
	}

	void setIntakeStoreRing(int state) {
		if (intakePart == 1) {
			botintake::setIntakeStoreRing(state);
		}
	}

	void setIntakeFilterOutColor(std::string colorText) {
		if (intakePart == 1) botintake::setFilterOutColor(colorText);
		else botintake2::setFilterOutColor(colorText);
	}

	/// @brief Set the state of Left Wing's pneumatic.
	/// @param state Expanded: true, retracted: false.
	/// @param delaySec Number of seconds to wait before setting the pneumatic state (in a task).
	void setGoalClampState(bool state, double delaySec) {
		goalclamp::setState(state, delaySec);
	}

	/// @brief Set the state of the lift's pneumatic.
	/// @param state Lifted: true, lowered: false
	void setIntakeLiftState(bool state) {
		IntakeLiftPneumatic.set(state);
	}

	void setArmHangState(int state, double delaySec) {
		botarmpneu::setState(state, delaySec);
	}

	void setArmStage(int stage, double delaySec) {
		botarm::setArmStage(stage, delaySec);
	}

	bool isArmResetted() {
		return botarm::isArmResetted();
	}

	void setSwingState(int state, double delaySec) {
		swing::setState(state, delaySec);
	}

	void setSwing2State(int state, double delaySec) {
		swing::set2ndState(state, delaySec);
	}
}
