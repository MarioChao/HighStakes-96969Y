#include "Mechanics/botDrive.h"
#include "Mechanics/botIntake.h"
#include "Mechanics/botLift.h"
// #include "Mechanics/botWings.h"
#include "Mechanics/goalClamp.h"
#include "Mechanics/botHang.h"
#include "Controller/controls.h"
#include "main.h"

namespace controls {
	void startThreads() {
		task intakeTast([] () -> int { botintake::startThread(); return 1; });
	}

	void setUpKeybinds() {
		Controller2.ButtonX.pressed([] () -> void {
			botdrive::switchDriveMode();
		});
		Controller1.ButtonL2.pressed([] () -> void {
			printf("Goal pneu: %d\n", GoalClampPneumatic.value());
			goalclamp::switchState();
		});
		Controller1.ButtonL1.pressed([] () -> void {
			printf("Intake lift pneu: %d\n", IntakeLiftPneumatic.value());
			botlift::switchState();
		});
		Controller1.ButtonUp.pressed([] () -> void {
			printf("Hang pneu: %d\n", HangPneumatic.value());
			HangPneumatic.set(!HangPneumatic.value());
			task::sleep(100);
			
		});
	}

	void preauton() {
		botdrive::preauton();
		goalclamp::preauton();
	}

	void resetStates() {

	}

	void doControls() {
        botdrive::control();
        botintake::control();
	}
}
