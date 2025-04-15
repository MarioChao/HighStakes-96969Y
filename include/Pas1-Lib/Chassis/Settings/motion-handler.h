#pragma once


namespace pas1_lib {
namespace chassis {
namespace settings {


struct MotionHandler {
	MotionHandler() {
		motionId = 0;
		isInMotionState = false;
	}

	void incrementMotion() { motionId++; }
	int getMotionId() { return motionId; }
	bool isRunningMotionId(int id) { return id == motionId; }

	bool getIsInMotion() { return isInMotionState; }
	void setIsInMotion(bool state) { isInMotionState = state; }
	void enterMotion() { setIsInMotion(true); }
	void exitMotion() { setIsInMotion(false); }

private:
	int motionId;
	bool isInMotionState;
};


}
}
}
