#include "Mechanics/botArm.h"
#include "Mechanics/botArmPneumatics.h"
#include "Mechanics/botDrive.h"
#include "Mechanics/botIntake.h"
#include "Mechanics/botIntake2.h"
#include "Mechanics/botIntakeLift.h"
#include "Mechanics/redirect.h"
#include "Mechanics/swing.h"
// #include "Mechanics/botWings.h"
#include "Controller/controls.h"
#include "Controller/rumble.h"
#include "Mechanics/goalClamp.h"
#include "Mechanics/climb-hook.h"
#include "Mechanics/climb-pto.h"
#include "Utilities/debugFunctions.h"

#include "Autonomous/autonFunctions.h"
#include "Autonomous/auton.h"

#include "Pas1-Lib/Chassis/Move/local-move-by.h"

#include "chassis-config.h"
#include "global-vars.h"
#include "main.h"


namespace {
using namespace pas1_lib::chassis::move;

int mode_manualArm = 0;
}


namespace controls {
void startThreads() {
	if (intakePartType == 1) {
		task intakeTask([]() -> int {
			botintake::runThread();
			return 1;
		});
	} else {
		task intakeTask([]() -> int {
			botintake2::runThread();
			return 1;
		});
	}
	task armTask([]() -> int {
		botarm::runThread();
		return 1;
	});

	task rumbleTask([]() -> int {
		rumble::runThread();
		return 1;
	});
	// rumble::setString(".");
}

void setUpKeybinds() {
	Controller2.ButtonX.pressed([]() -> void { botdrive::switchDriveMode(); });
	Controller2.ButtonY.pressed([]() -> void { botarm::resetArmEncoder(); });
	Controller2.ButtonA.pressed([]() -> void {
		botdrive::setControlState(false);
		botintake::setControlState(false);

		auton::runAutonomous();

		botdrive::setControlState(true);
		botintake::setControlState(true);
		botdrive::preauton();
		botintake::preauton();
	});


	/* ---------- Mode switch ---------- */

	// Manual arm mode
	// Controller1.ButtonUp.pressed([]() -> void {
	// 	mode_manualArm++;
	// 	mode_manualArm %= 2;

	// 	if (mode_manualArm == 0) debug::printOnController("automatic arm");
	// 	else debug::printOnController("manual arm");

	// 	rumble::setConstantRumbling(false);
	// 	rumble::setString(".");
	// });


	// Controller 1

	/* Arm stages */
	// Stage 0
	Controller1.ButtonRight.pressed([]() -> void {
		if (!botarm::isArmResetted()) return;

		botarm::setArmStage(0);
		botintake::setColorFiltering(true);
		// botintake::setColorFiltering(false);
	});
	// Stage 1 or 2
	Controller1.ButtonB.pressed([]() -> void {
		if (!botarm::isArmResetted()) return;

		botarm::setArmStage(1);
		botintake::setColorFiltering(false);
	});
	// Stage 3
	Controller1.ButtonX.pressed([]() -> void {
		if (!botarm::isArmResetted()) return;

		botarm::setArmStage(3);
		botintake::setColorFiltering(true);
		// botintake::setColorFiltering(false);
	});
	// Stage 0 or 4 (descore)
	Controller1.ButtonLeft.pressed([]() -> void {
		if (!botarm::isArmResetted()) return;

		if (botarm::getArmStage() <= 2) {
			task reverseIntake([]() -> int {
				botintake::setControlState(false);
				botintake::setState(-1);
				wait(0.3, sec);
				botintake::setState(0);
				botintake::setControlState(true);
				return 1;
			});
		}
		if (botarm::getArmStage() == 4) botarm::setArmStage(0);
		else botarm::setArmStage(4);
		botintake::setColorFiltering(true);
		// botintake::setColorFiltering(false);
	});
	// Stage 6 (dunk aiming angle)
	Controller1.ButtonUp.pressed([]() -> void {
		if (!botarm::isArmResetted()) return;

		if (botarm::getArmStage() <= 2) {
			task reverseIntake([]() -> int {
				botintake::setControlState(false);
				botintake::setState(-1);
				wait(0.3, sec);
				botintake::setState(0);
				botintake::setControlState(true);
				return 1;
			});
		}
		botarm::setArmStage(6);
	});
	// Stage 0 or 9 (score wall stake / tip goal)
	Controller1.ButtonL1.pressed([]() -> void {
		if (mode_manualArm != 0) return;

		if (!botarm::isArmResetted()) return;

		if (botarm::getArmStage() <= 2) {
			task reverseIntake([]() -> int {
				botintake::setControlState(false);
				botintake::setState(-1);
				wait(0.3, sec);
				botintake::setState(0);
				botintake::setControlState(true);
				return 1;
			});
		}
		if (botarm::getArmStage() == 9) botarm::setArmStage(0);
		else botarm::setArmStage(9);
		botintake::setColorFiltering(true);
		// botintake::setColorFiltering(false);
	});

	/* Ring color filter */
	Controller2.ButtonR1.pressed([]() -> void {
		botintake::setIntakeStoreRing(true);
	});
	Controller2.ButtonUp.pressed([]() -> void {
		rumble::setConstantRumbling(false);
		rumble::setString(".");
		if (intakePartType == 1) botintake::switchFilterColor();
		else botintake2::switchFilterColor();
	});

	/* Macro */
	Controller2.ButtonB.pressed([]() -> void {
		botdrive::setControlState(false);
		botintake::setControlState(false);

		// autonfunctions::pid_diff::driveDistanceTiles(0.1);
		local::driveAndTurn(robotChassis, local::driveAndTurn_params(0.1, robotChassis.getLookPose().getRotation()), false);

		botdrive::setControlState(true);
		botintake::setControlState(true);
		botdrive::preauton();
		botintake::preauton();
	});

	/* Pneumatics */
	// Swing
	Controller2.ButtonLeft.pressed([]() -> void {
		if (Controller2.ButtonA.pressing()) {
			printf("Left pneu: %ld\n", LeftSword_pneumatics.value());
			swing::switchState_left();
		}
	});
	Controller2.ButtonRight.pressed([]() -> void {
		if (Controller2.ButtonA.pressing()) {
			printf("Right pneu: %ld\n", RightSword_pneumatics.value());
			swing::switchState_right();
		}
	});
	// Clamp
	Controller1.ButtonL2.pressed([]() -> void {
		if (mode_manualArm != 0) return;

		printf("Goal pneu: %ld\n", GoalClampPneumatic.value());
		goalclamp::switchState();
	});
	// Intake lift
	Controller2.ButtonL1.pressed([]() -> void {
		botintakelift::switchState();
	});
	// Climbing PTO
	Controller2.ButtonB.pressed([]() -> void {
		climb_pto::switchState();
	});
	// Moving hook
	Controller2.ButtonY.pressed([]() -> void {
		climb_hook::switchState();
	});
}

void preauton() {
	botdrive::preauton();
	botintake::preauton();
	botarm::preauton();
	goalclamp::preauton();
}

void resetStates() {
	LeftRightMotors.setStopping(brake);

	// Reset arm encoder
	if (!botarm::isArmResetted()) {
		task resetArm([]() -> int {
			botarm::resetArmEncoder();
			return 1;
		});
	}
	botarm::setArmStage(0);
	botintake::setAntiJam(false);
	botintake::setIntakeStoreRing(0);
	botintake::setColorFiltering(true);
	swing::setState_left(0);
	swing::set2ndState_left(0);
	swing::setState_right(0);
	swing::set2ndState_right(0);
}

void doControls() {
	botdrive::control();
	if (intakePartType == 1) {
		botintake::control(
			(int) Controller1.ButtonR2.pressing() -
			(int) Controller1.ButtonR1.pressing(),
			/*This is not used =>*/ (int) Controller1.ButtonX.pressing());
	} else {
		botintake2::control(
			(int) Controller1.ButtonR1.pressing() -
			(int) Controller1.ButtonR2.pressing(),
			/*This is not used =>*/ (int) Controller1.ButtonX.pressing());
	}
	if (mode_manualArm == 1) {
		botarm::control(
			(int) Controller1.ButtonL1.pressing() -
			(int) Controller1.ButtonL2.pressing()
		);
	}
}
}  // namespace controls
