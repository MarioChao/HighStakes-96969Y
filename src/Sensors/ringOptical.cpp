#include "Sensors/ringOptical.h"
#include "main.h"

namespace {
	void ringOpticalThread();

	bool isDetectingRing;
	std::string detectedRingColor;
}

namespace ringoptical {
	void startThread() {
		task ringOpticalTask([]() -> int {
			ringOpticalThread();
			return 1;
		});
	}

	bool isDetecting() {
		return isDetectingRing;
	}

	std::string getDetectedColor() {
		return detectedRingColor;
	}
}

namespace {
	void ringOpticalThread() {
		while (true) {
			// Update detecting ring
			isDetectingRing = RingOpticalSensor.isNearObject();
			if (isDetectingRing) {
				// Update detected ring color
				if (RingOpticalSensor.hue() <= 45 || RingOpticalSensor.hue() >= 340) {
					detectedRingColor = "red";
					// debug::printOnController("Red ring");
				} else if (140 <= RingOpticalSensor.hue() && RingOpticalSensor.hue() <= 230) {
					detectedRingColor = "blue";
					// debug::printOnController("Blue ring");
				} else {
					detectedRingColor = "none";
					// debug::printOnController("No ring");
				}
			}

			wait(3, msec);
		}
	}
}
