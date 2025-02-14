#pragma once

#include <iostream>

namespace ringoptical {
	void startThread();

	void updateDetection();
	bool isDetecting();
	std::string getDetectedColor();
}
