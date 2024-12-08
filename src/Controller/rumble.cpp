#include "Controller/rumble.h"

#include "main.h"

namespace {
	std::string nextRumbleString = "";

	bool keepRumble = false;
}

namespace rumble {
	void runThread() {
		while (true) {
			// Thread code here
			if (keepRumble) {
				Controller1.rumble(".");
			} else if (nextRumbleString != "") {
				Controller1.rumble(nextRumbleString.c_str());
				nextRumbleString = "";
			} else {
				;
			}

			wait(200, msec);
		}
	}

	void setConstantRumbling(bool willRumble) {
		keepRumble = willRumble;
	}

	void setString(std::string rumbleString) {
		nextRumbleString = rumbleString;
	}
}
