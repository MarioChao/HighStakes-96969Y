#include "Mechanics/botWings.h"
#include "main.h"

namespace {
	void setWingsState(bool);
	void changeFrontWingsState();
	void changeBothWingsToSameState();
	void changeBothWingsToDifferentState();
	void changeLeftWingState();
	void changeRightWingState();

	double frontWingsDebounce = false;
	double wingsDebounce = false;
	double leftWingDebounce = false;
	double rightWingDebounce = false;
}

namespace botwings {
	void changeFrontWingsState() {
		::changeFrontWingsState();
	}

	void changeBothWingsToSameState() {
		::changeBothWingsToSameState();
	}

	void changeBothWingsToDifferentState() {
		::changeBothWingsToDifferentState();
	}

	void changeLeftWingState() {
		::changeLeftWingState();
	}

	void changeRightWingState() {
		::changeRightWingState();
	}

	void setWingsState(bool state) {
		::setWingsState(state);
	}
}

namespace {
	void setWingsState(bool state) {
		LeftWingPneumatic.set(state);
		RightWingPneumatic.set(state);
	}
	void changeFrontWingsState() {
		if (!frontWingsDebounce) {
			frontWingsDebounce = true;

			int oldValue = FrontWingsPneumatic.value();
			int newValue = oldValue ^ 1;
			FrontWingsPneumatic.set(newValue);
			task::sleep(50);

			frontWingsDebounce = false;
		}
	}
	void changeBothWingsToSameState() {
		if (!wingsDebounce) {
			wingsDebounce = true;

			int oldValue = LeftWingPneumatic.value() && RightWingPneumatic.value();
			int newValue = oldValue ^ 1;
			setWingsState(newValue);
			task::sleep(50);

			wingsDebounce = false;
		}
	}
	void changeBothWingsToDifferentState() {
		if (!wingsDebounce) {
			wingsDebounce = true;

			int oldValue1 = LeftWingPneumatic.value();
			int oldValue2 = RightWingPneumatic.value();
			int newValue1 = oldValue1 ^ 1;
			int newValue2 = oldValue2 ^ 1;
			LeftWingPneumatic.set(newValue1);
			RightWingPneumatic.set(newValue2);
			task::sleep(50);

			wingsDebounce = false;
		}
	}
	void changeLeftWingState() {
		if (!leftWingDebounce) {
			leftWingDebounce = true;

			int oldValue = LeftWingPneumatic.value();
			int newValue = oldValue ^ 1;
			LeftWingPneumatic.set(newValue);
			task::sleep(50);

			leftWingDebounce = false;
		}
	}
	void changeRightWingState() {
		if (!rightWingDebounce) {
			rightWingDebounce = true;

			int oldValue = RightWingPneumatic.value();
			int newValue = oldValue ^ 1;
			RightWingPneumatic.set(newValue);
			task::sleep(50);

			rightWingDebounce = false;
		}
	}
}
