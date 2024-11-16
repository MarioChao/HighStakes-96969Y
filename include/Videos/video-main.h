#pragma once

namespace video {
	void keybindVideos();
	void startThread();

	void switchVideoState(int increment = 1);
	int getCurrentVideoId();
}
