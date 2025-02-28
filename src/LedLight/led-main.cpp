#include "LedLight/led-main.h"
#include "Utilities/generalUtility.h"
#include "Graphics/GraphicMain.h"
#include "Graphics/BrainScreen.h"
#include "global-vars.h"
#include "main.h"

namespace {
	triport::port rPort = Brain.ThreeWirePort.B;
	triport::port gPort = Brain.ThreeWirePort.A;
	triport::port bPort = Brain.ThreeWirePort.C;

	digital_out ledR(rPort);
	digital_out ledG(gPort);
	digital_out ledB(bPort);
	pwm_out ledLicense(Brain.ThreeWirePort.D);

	double stageTime_seconds[3] = {75, 95, 105};
	// double stageTime_seconds[3] = {5, 10, 15};

	void ledLightThread();

	void ledLicenseThread();
	void setLicenseFade(double state_pct, double duration_seconds);

	double licenseFadeRange_pct[2] = {0, 0};
	double licenseFadeRange_durationSec[2] = {0, 1};
	double licenseState_pct = 0;
}

namespace ledlight {
	void startThread() {
		task ledLightTask([]() -> int {
			ledLightThread();
			return 1;
		});
		task ledLicenseTask([]() -> int {
			ledLicenseThread();
			return 1;
		});
	}

	void showColor(bool r, bool g, bool b) {
		ledR.set(r);
		ledG.set(g);
		ledB.set(b);
	}

	void setLicenseState(double state_pct) {
		ledLicense.state(state_pct, pct);
	}
}

namespace {
	void ledLightThread() {
		bool rgb[3] = {0, 0, 0};
		double delayMs = 50;
		double licenseState_pct = 0;
		while (true) {
			// Set color configs
			if (drivingTimer.time(seconds) < stageTime_seconds[0]) {
				// Random color
				int val = rand() % 7 + 1;
				rgb[0] = val & 4;
				rgb[1] = val & 2;
				rgb[2] = val & 1;
				delayMs = 300;
			} else if (drivingTimer.time(seconds) < stageTime_seconds[1]) {
				// No green
				rgb[1] = 0;

				// Red-blue swap
				if (rgb[2]) {
					rgb[0] = 1;
					rgb[2] = 0;
				} else {
					rgb[0] = 0;
					rgb[2] = 1;
				}
				delayMs = genutil::rangeMap(drivingTimer.time(seconds), stageTime_seconds[0], stageTime_seconds[1], 500, 50);
			} else if (drivingTimer.time(seconds) < stageTime_seconds[2]) {
				// No green and blue
				rgb[1] = rgb[2] = 0;

				// Flashing red
				rgb[0] ^= 1;
				delayMs = 50;
			} else {
				// Eternal red
				rgb[0] = rgb[2] = 1;
				rgb[1] = 0;
				delayMs = 1000;
			}

			// Show color
			ledlight::showColor(rgb[0], rgb[1], rgb[2]);
			gfxmain::setClearColor(color(rgb[0] * 255, rgb[1] * 255, rgb[2] * 255));
			brainscreen::redraw();

			// License
			if (licenseState_pct > 0) licenseState_pct = -100;
			else licenseState_pct = 100;
			setLicenseFade(licenseState_pct, delayMs / 1000.0);

			wait(delayMs, msec);
		}
	}

	void ledLicenseThread() {
		while (true) {
			licenseState_pct = genutil::rangeMap(
				drivingTimer.time(seconds), licenseFadeRange_durationSec[0], licenseFadeRange_durationSec[1],
				licenseFadeRange_pct[0], licenseFadeRange_pct[1]
			);
			licenseState_pct = genutil::clamp(licenseState_pct, -100, 100);
			ledlight::setLicenseState(licenseState_pct);
			wait(10, msec);
		}
	}

	void setLicenseFade(double state_pct, double duration_seconds) {
		licenseFadeRange_pct[0] = licenseState_pct;
		licenseFadeRange_pct[1] = state_pct;
		licenseFadeRange_durationSec[0] = drivingTimer.time(seconds);
		licenseFadeRange_durationSec[1] = licenseFadeRange_durationSec[0] + duration_seconds;
	}
}
