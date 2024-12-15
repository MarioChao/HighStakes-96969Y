#include "Mechanics/botArm.h"
#include "Mechanics/botArmPneumatics.h"
#include "Mechanics/botDrive.h"
#include "Mechanics/botIntake.h"
#include "Mechanics/botIntake2.h"
#include "Mechanics/botLift.h"
#include "Mechanics/redirect.h"
#include "Mechanics/swing.h"
// #include "Mechanics/botWings.h"
#include "Controller/controls.h"
#include "Controller/rumble.h"
#include "Mechanics/goalClamp.h"
#include "Utilities/debugFunctions.h"
#include "main.h"

namespace controls {
	void startThreads() {
		if (intakePart == 1) {
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
		rumble::setString(".");
	}

	void setUpKeybinds() {
		Controller2.ButtonX.pressed([]() -> void { botdrive::switchDriveMode(); });
		Controller2.ButtonY.pressed([]() -> void { botarm::resetArmEncoder(); });

		// Controller 1
		Controller1.ButtonX.pressed([]() -> void {
			if (!botarm::isArmResetted()) {
				return;
			}
			botarm::setArmStage(0);
			botintake::setColorFiltering(true);
		});
		// Controller1.ButtonY.pressed([]() -> void {
		// 	if (intakePart == 1) {
		// 		if (botintake::getIntakeVelocity() == 100) {
		// 			botintake::setIntakeVelocity(80);
		// 		} else {
		// 			botintake::setIntakeVelocity(100);
		// 		}
		// 	}
		// });
		Controller1.ButtonY.pressed([]() -> void {
			rumble::setConstantRumbling(false);
			rumble::setString(".");
			if (intakePart == 1) botintake::switchFilterColor();
			else botintake2::switchFilterColor();
		});
		Controller1.ButtonA.pressed([]() -> void {
			swing::switchState();
		});
		Controller1.ButtonB.pressed([]() -> void {
			if (intakePart == 1) {
				if (!botarm::isArmResetted()) {
					return;
				}

				if (botintake::isColorFiltering()) {
					botintake::setColorFiltering(false);
					redirect::setState(1);
					botarm::setArmStage(1);
					task closeRedirect([]() -> int {
						wait(5, sec);
						botintake::setColorFiltering(true);
						return 1;
					});
				} else {
					botarm::setArmStage(0);
					botintake::setColorFiltering(true);
				}
			}
		});
		Controller1.ButtonL2.pressed([]() -> void {
			printf("Goal pneu: %ld\n", GoalClampPneumatic.value());
			goalclamp::switchState();
		});
		Controller1.ButtonL1.pressed([]() -> void {
			// if (botarmpneu::pressedCount < 14 || drivingTimer.value() > 105 - 15) {
			// 	botarmpneu::switchState();
			// }
			if (!botarm::isArmResetted()) {
				return;
			}

			// Neutral wall stake
			botarm::setArmStage(3);
			botintake::setColorFiltering(true);
		});
		Controller1.ButtonUp.pressed([]() -> void {
			if (intakePart == 1) {
				if (!botarm::isArmResetted()) {
					return;
				}
				// botintake::switchMode();
				// Alliance wall stake
				botarm::setArmStage(2);
				botintake::setColorFiltering(true);
			} else {
				botintake2::switchMode();
			}
		});
		Controller1.ButtonDown.pressed([]() -> void {
			// if (botdrive::getMaxDriveVelocity() >= 99.0) {
			// 	botdrive::setMaxDriveVelocity(50.0);
			// 	debug::printOnController("50\% drive speed");
			// } else {
			// 	botdrive::setMaxDriveVelocity(100.0);
			// 	debug::printOnController("100\% drive speed");
			// }
			if (intakePart == 1) {
				botintake::setFilterOutColor("none");
				debug::printOnController("filter none");
				rumble::setConstantRumbling(true);
			}
		});
		Controller1.ButtonLeft.pressed([]() -> void {
			swing::switch2ndState();
		});
	}

	void preauton() {
		botdrive::preauton();
		botarm::preauton();
		goalclamp::preauton();
	}

	void resetStates() {
		LeftRightMotors.setStopping(brake);

		// Reset arm encoder
		if (!botarm::isArmResetted()) {
			task resetArm([] () -> int {
				botarm::resetArmEncoder();
				return 1;
			});
		}
		botintake::setIntakeStoreRing(0);
		swing::setState(0);
		swing::set2ndState(0);
	}

	void doControls() {
		botdrive::control();
		if (intakePart == 1) {
			botintake::control(
				(int)Controller1.ButtonR1.pressing() -
				(int)Controller1.ButtonR2.pressing(),
				/*This is not used =>*/ (int)Controller1.ButtonX.pressing());
		} else {
			botintake2::control(
				(int)Controller1.ButtonR1.pressing() -
				(int)Controller1.ButtonR2.pressing(),
				/*This is not used =>*/ (int)Controller1.ButtonX.pressing());
		}
		// botarm::control((int)Controller1.ButtonL1.pressing() -
		// 				(int)Controller1.ButtonDown.pressing());
	}
}  // namespace controls
