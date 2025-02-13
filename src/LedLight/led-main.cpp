#include "LedLight/led-main.h"
#include "Utilities/generalUtility.h"
#include "Graphics/GraphicMain.h"
#include "Graphics/BrainScreen.h"
#include "main.h"

namespace {
	triport::port rPort = Brain.ThreeWirePort.B;
	triport::port gPort = Brain.ThreeWirePort.A;
	triport::port bPort = Brain.ThreeWirePort.C;

	digital_out ledR(rPort);
	digital_out ledG(gPort);
	digital_out ledB(bPort);

	double stageTimeSeconds[3] = {75, 95, 105};
	// double stageTimeSeconds[3] = {5, 10, 15};

	void ledLightThread();
}

namespace ledlight {
	void startThread() {
		task ledTask([]() -> int {
			ledLightThread();
			return 1;
		});
	}

	void showColor(bool r, bool g, bool b) {
		ledR.set(r);
		ledG.set(g);
		ledB.set(b);
	}
}

namespace {
	void ledLightThread() {
		bool rgb[3] = {0, 0, 0};
		double delayMs = 50;
		while (true) {
			// Set configs
			if (drivingTimer.time(seconds) < stageTimeSeconds[0]) {
				// Random color
				int val = rand() % 7 + 1;
				rgb[0] = val & 4;
				rgb[1] = val & 2;
				rgb[2] = val & 1;
				delayMs = 300;
			} else if (drivingTimer.time(seconds) < stageTimeSeconds[1]) {
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
				delayMs = genutil::rangeMap(drivingTimer.time(seconds), stageTimeSeconds[0], stageTimeSeconds[1], 500, 50);
			} else if (drivingTimer.time(seconds) < stageTimeSeconds[2]) {
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
			wait(delayMs, msec);
		}
	}
}
