#pragma once

#include <iostream>

namespace ringoptical {
	void startThread();

	bool isDetecting();
	std::string getDetectedColor();
}
