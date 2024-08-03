#include "Mechanics/botHang.h"
#include "main.h"
extern int _taskState;
extern double _taskDelay;
namespace {
    void setHangState(bool);
    void switchhangState();

    bool HangDebounce = false;

    int hangState = 0;
}

namespace {
    void setHangState(bool value) {
        hangState = value;
        HangPneumatic.set(hangState);
    }

    /// @brief Change the lift's position to high or low.
    void switchhangState() {
        if (!HangDebounce) {
            HangDebounce = true;

            setHangState(hangState ^ 1);
            task::sleep(10);

            HangDebounce = false;
        }
    }

    void setState(int state, double delaySec = 0.0) {
		// Set global variables
		_taskState = state;
		_taskDelay = delaySec;

		task setState([] () -> int {
			// Get global variables
			int taskState = _taskState;
			double taskDelay = _taskDelay;

			// Delay setting state
			if (taskDelay > 1e-9) {
				task::sleep(taskDelay * 1000);
			}

			// Set state here
			printf("New state: %d\n", taskState);
			HangPneumatic.set(taskState);

			return 1;
		});
	}

	void switchState() {
		setState(!HangPneumatic.value());
	}
}
