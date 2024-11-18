#include "Mechanics/botDrive.h"
#include "Mechanics/botIntake.h"
#include "Mechanics/botIntake2.h"
#include "Mechanics/botLift.h"
#include "Mechanics/botArm.h"
#include "Mechanics/botArmPneumatics.h"
#include "Mechanics/swing.h"
// #include "Mechanics/botWings.h"
#include "Mechanics/goalClamp.h"
#include "Utilities/debugFunctions.h"

#include "Controller/controls.h"
#include "main.h"

namespace controls {
	void startThreads() {
		if (intakePart == 1) {
			task intakeTask([]() -> int { botintake::runThread(); return 1; });
		} else {
			task intakeTask([]() -> int { botintake2::runThread(); return 1; });
		}
		// task armTask([] () -> int { botarm::runThread(); return 1; });
	}

	void setUpKeybinds() {
		Controller2.ButtonX.pressed([]() -> void {
			botdrive::switchDriveMode();
		});

		// Controller 1
		Controller1.ButtonX.pressed([]() -> void {
			if (intakePart == 1) botintake::switchMode();
			else botintake2::switchMode();
		});
		Controller1.ButtonY.pressed([]() -> void {
			if (intakePart == 1) {
				if (botintake::getIntakeVelocity() == 100) {
					botintake::setIntakeVelocity(80);
				} else {
					botintake::setIntakeVelocity(100);
				}
			}
		});
		Controller1.ButtonA.pressed([]() -> void {
			swing::switchState();
		});
		Controller1.ButtonB.pressed([]() -> void {
			if (intakePart == 1) botintake::switchFilterColor();
			else botintake2::switchFilterColor();
		});
		Controller1.ButtonL2.pressed([]() -> void {
			printf("Goal pneu: %d\n", GoalClampPneumatic.value());
			goalclamp::switchState();
		});
		Controller1.ButtonL1.pressed([]() -> void {
			if (botarmpneu::pressedCount < 14 || drivingTimer.value() > 105 - 15) {
				botarmpneu::switchState();
			}
		});
		Controller1.ButtonUp.pressed([]() -> void {
			if (botdrive::getMaxDriveVelocity() >= 99.0) {
				botdrive::setMaxDriveVelocity(50.0);
				debug::printOnController("50\% drive speed");
			} else {
				botdrive::setMaxDriveVelocity(100.0);
				debug::printOnController("100\% drive speed");
			}
		});

	}

	void preauton() {
		botdrive::preauton();
		// botarm::preauton();
		goalclamp::preauton();
	}

	void resetStates() {
		LeftRightMotors.setStopping(brake);
	}

	void doControls() {
		botdrive::control();
		if (intakePart == 1) {
			botintake::control(
				(int) Controller1.ButtonR1.pressing() - (int) Controller1.ButtonR2.pressing(),
				/*This is not used =>*/ (int) Controller1.ButtonX.pressing()
			);
		} else {
			botintake2::control(
				(int) Controller1.ButtonR1.pressing() - (int) Controller1.ButtonR2.pressing(),
				/*This is not used =>*/ (int) Controller1.ButtonX.pressing()
			);
		}
		// botarm::control((int) Controller1.ButtonUp.pressing() - (int) Controller1.ButtonDown.pressing());
	}
}
