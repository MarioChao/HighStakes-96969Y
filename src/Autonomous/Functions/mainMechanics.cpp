#include "Autonomous/autonFunctions.h"

#include "Aespa-Lib/Winter-Utilities/angle.h"
#include "Mechanics/botIntake.h"
#include "Mechanics/botIntake2.h"
#include "Mechanics/botArm.h"
#include "Mechanics/botArmPneumatics.h"
#include "Mechanics/swing.h"
#include "Mechanics/goalClamp.h"
#include "Mechanics/redirect.h"

#include "global-vars.h"
#include "chassis-config.h"
#include "main.h"


namespace autonfunctions {

void setRobotLookPose(aespa_lib::datas::Linegular pose) {
	robotChassis.setLookPose(pose);
	if (mainUseSimulator) {
		robotSimulator.position = Vector3(pose.getX(), pose.getY());
		robotSimulator.angularPosition = pose.getRotation().polarRad();
	}
}

void setRobotPosition(double x_tiles, double y_tiles) {
	mainOdometry.setPosition_scaled(x_tiles, y_tiles);
	if (mainUseSimulator) robotSimulator.position = Vector3(x_tiles, y_tiles);
}

/// @brief Set the look field absolute angle reading to a specified value. Doesn't turn the robot.
/// @param fieldAngle_degrees The angle (in degrees) to be set for the current orientation.
void setRobotRotation(double fieldAngle_degrees) {
	mainOdometry.setLookAngle_field(fieldAngle_degrees);
	if (mainUseSimulator) robotSimulator.angularPosition = aespa_lib::genutil::toRadians(aespa_lib::angle::swapFieldPolar_degrees(fieldAngle_degrees));
}

timer _autonTimer;

void setDifferentialUseRelativeRotation(bool useRelativeRotation) {
	robotChassis.autonSettings.useRelativeRotation = useRelativeRotation;
}

/// @brief Set the state of the intake.
/// @param state Forward: 1, released: 0, reversed: -1
/// @param delaySec Number of seconds to wait before setting the state (in a task).
void setIntakeState(int state, double delaySec) {
	if (intakePartType == 1) botintake::setState(state, delaySec);
	else botintake2::setState(state, delaySec);
}


/// @brief Set the state of the top intake.
/// @param state Forward: 1, released: 0, reversed: -1
/// @param delaySec Number of seconds to wait before setting the state (in a task).
void setIntakeTopState(int state, double delaySec) {
	if (intakePartType == 1) return;
	else botintake2::setState2(state, delaySec);
}


/// @brief Set the state of the bottom intake.
/// @param state Forward: 1, released: 0, reversed: -1
/// @param delaySec Number of seconds to wait before setting the state (in a task).
void setIntakeBottomState(int state, double delaySec) {
	if (intakePartType == 1) return;
	else botintake2::setState3(state, delaySec);
}

void setIntakeAntiJam(bool state) {
	botintake::setAntiJam(state);
}

/// @brief Set the hook mode of the intake.
/// @param state Normal: 0, to arm: 1
void setIntakeToArm(int state, double delaySec) {
	if (intakePartType == 1) {
		if (state) {
			botintake::setColorFiltering(false, delaySec);
			redirect::setState(1, delaySec);
			botarm::setArmStage(1, delaySec);
		} else {
			botintake::setColorFiltering(true, delaySec);
		}
	} else botintake2::setHookMode(state);
}

void setIntakeStoreRing(int state, double delaySec) {
	if (intakePartType == 1) {
		botintake::setIntakeStoreRing(state, delaySec);
	}
}

void setIntakeFilterOutColor(std::string colorText) {
	if (intakePartType == 1) botintake::setFilterOutColor(colorText);
	else botintake2::setFilterOutColor(colorText);
}

void setIntakeFilterEnabled(bool state, double delaySec) {
	if (intakePartType == 1) botintake::setColorFiltering(state, delaySec);
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

void setArmStage(int stageId, double delay_sec, double maxSpeed_pct) {
	botarm::setArmStage(stageId, delay_sec, maxSpeed_pct);
}

bool isArmResetted() {
	return botarm::isArmResetted();
}

void setArmResetDefaultStage(int stageId) {
	botarm::setResetDefaultStage(stageId);
}

void setSwingState_left(int state, double delaySec) { swing::setState_left(state, delaySec); }
void setSwing2State_left(int state, double delaySec) { swing::set2ndState_left(state, delaySec); }

void setSwingState_right(int state, double delaySec) { swing::setState_right(state, delaySec); }
void setSwing2State_right(int state, double delaySec) { swing::set2ndState_right(state, delaySec); }

}
