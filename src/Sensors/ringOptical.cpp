#include "Sensors/ringOptical.h"
#include "main.h"

namespace {
	void ringOpticalThread();
	void updateDetection();

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

	void updateDetection() {
		::updateDetection();
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
			updateDetection();

			wait(10, msec);
		}
	}

	void updateDetection() {
		bool isNear = RingOpticalSensor.isNearObject();
		if (isNear) {
			// Update detected ring color
			if (RingOpticalSensor.hue() <= 45 || RingOpticalSensor.hue() >= 340) {
				detectedRingColor = "red";
				isDetectingRing = true;
				// debug::printOnController("Red ring");
			} else if (140 <= RingOpticalSensor.hue() && RingOpticalSensor.hue() <= 230) {
				detectedRingColor = "blue";
				isDetectingRing = true;
				// debug::printOnController("Blue ring");
			} else {
				detectedRingColor = "none";
				isDetectingRing = false;
				// debug::printOnController("No ring");
			}
		} else {
			detectedRingColor = "none";
			isDetectingRing = false;
		}
	}
}
