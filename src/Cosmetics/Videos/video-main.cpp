#include "Cosmetics/Videos/video-main.h"
#include "Cosmetics/Videos/video-info.h"
#include "Cosmetics/Videos/video-objects.h"
#include "Gfx-Lib/GraphicMain.h"
#include "main.h"

namespace {
	// Functions

	void brainVideosThread();

	void drawVideos();

	// Variables

	std::vector< std::vector<VideoInfo *> > videoObjects = {
		{},
		{&teamLogo},
		{&madotsuki, &madotsuki, &madotsuki, &madotsuki},
		{&ningning3, &ningning3, &ningning3},
	};
	std::vector< std::vector< std::pair<int, int> > > videoObjectPositions = {
		{},
		{{0, 0}},
		{{0, 0}, {120, 0}, {240, 0}, {360, 0}},
		{{0, 0}, {167, 60}, {345, 120}},
	};

	int playingVideoId = 0;
	int refreshedVideoId = -1;

	bool videoDebounce = false;

	vex::timer videoTimePosition;
}

namespace video {
	void keybindVideos() {
		// Controller1.ButtonLeft.pressed([]() -> void {
		// 	switchVideoState(-1);
		// });
		Controller1.ButtonRight.pressed([]() -> void {
			switchVideoState(1);
		});
		Brain.Screen.pressed([]() -> void {
			if (playingVideoId == 0) return;
			switchVideoState(1);
		});
	}

	void startThread() {
		task brainVideos([]() -> int {
			brainVideosThread();
			return 1;
		});
	}

	void switchVideoState(int increment) {
		if (!videoDebounce) {
			videoDebounce = true;

			// Increment video id
			playingVideoId += increment;
			playingVideoId %= (int) videoObjects.size();
			if (playingVideoId < 0) {
				playingVideoId += (int) videoObjects.size();
			}

			// if (playingVideoId > 0) {
			// 	printf("Playing video %d!\n", playingVideoId);
			// }

			// Refresh screen
			refreshedVideoId = -1;
			task::sleep(30);
			Brain.Screen.clearScreen(gfxmain::getClearColor());
			task::sleep(30);
			refreshedVideoId = playingVideoId;

			// Reset time
			videoTimePosition.reset();

			videoDebounce = false;
		}
	}

	int getCurrentVideoId() {
		return refreshedVideoId;
	}
}

namespace {
	void brainVideosThread() {
		video::switchVideoState(0);
		while (true) {
			if (playingVideoId > 0) {
				drawVideos();
			}
			task::sleep(20);
		}
	}

	void drawVideos() {
		for (int i = 0; i < (int) videoObjects[playingVideoId].size(); i++) {
			VideoInfo *video = videoObjects[playingVideoId][i];
			std::pair<int, int> videoPosition = videoObjectPositions[playingVideoId][i];
			video->setFrameId(videoTimePosition.time(timeUnits::msec));
			video->drawFrame(videoPosition.first, videoPosition.second);
		}
	}
}
