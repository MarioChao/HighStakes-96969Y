#include "Sensors/inertial-s.h"
#include "main.h"

namespace {
	void inertialSThread();

	double lastAngle = -1e9;
	bool isStable = false;
}

namespace inertial_s {
	void startThread() {
		task inertialSTask([]() -> int {
			inertialSThread();
			return 1;
		});
	}

	bool isStable() {
		return ::isStable;
	}
}

namespace {
	void inertialSThread() {
		while (true) {
			double currentAngle = InertialSensor.angle(deg);
			if (fabs(currentAngle - lastAngle) < 0.05) {
				isStable = true;
			} else {
				isStable = false;
			}
			lastAngle = currentAngle;
			wait(1500, msec);
		}
	}
}
