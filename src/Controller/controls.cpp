#include "Mechanics/botDrive.h"
#include "Mechanics/botIntake.h"
#include "Mechanics/botLift.h"
#include "Mechanics/botArm.h"
#include "Mechanics/botArmPneumatics.h"
#include "Mechanics/swing.h"
// #include "Mechanics/botWings.h"
#include "Mechanics/goalClamp.h"

#include "Controller/controls.h"
#include "main.h"

namespace controls {
	void startThreads() {
		task intakeTask([] () -> int { botintake::runThread(); return 1; });
		// task armTask([] () -> int { botarm::runThread(); return 1; });
	}

	void setUpKeybinds() {
		Controller2.ButtonX.pressed([] () -> void {
			botdrive::switchDriveMode();
		});
		Controller1.ButtonX.pressed([] () -> void {
			botintake::switchMode();
		});
		Controller1.ButtonL2.pressed([] () -> void {
			printf("Goal pneu: %d\n", GoalClampPneumatic.value());
			goalclamp::switchState();
		});
		Controller1.ButtonL1.pressed([] () -> void {
			if (botarmpneu::pressedCount < 14 || drivingTimer.value() > 105 - 15) {
				botarmpneu::switchState();
			}
		});
		Controller1.ButtonB.pressed([] () -> void {
			botintake::switchFilterColor();
		});
		Controller1.ButtonDown.pressed([] () -> void {
			swing::switchState();
		});
		
	}

	void preauton() {
		botdrive::preauton();
		// botarm::preauton();
		goalclamp::preauton();
	}

	void resetStates() {

	}

	void doControls() {
		botdrive::control();
		botintake::control(
			(int) Controller1.ButtonR1.pressing() - (int) Controller1.ButtonR2.pressing(),
			/*This is not used =>*/ (int) Controller1.ButtonX.pressing()
		);
		// botarm::control((int) Controller1.ButtonUp.pressing() - (int) Controller1.ButtonDown.pressing());
	}
}
