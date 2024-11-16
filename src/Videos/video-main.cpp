#include "Videos/video-main.h"
#include "Videos/video-info.h"
#include "Videos/video-objects.h"
#include "main.h"

namespace {
	// Functions

	void brainVideosThread();

	void drawVideos();

	// Variables

	std::vector< std::vector<VideoInfo *> > videoObjects = {
		{},
		{&teamLogo},
		{&yoruNiKakeru, &badApple_suika, &yoruNiKakeru, &badApple_suika},
		{&badApple_image},
		{&marioFlag},
		{&ningning1, &ningning2, &ningning3},
	};
	std::vector< std::vector< std::pair<int, int> > > videoObjectPositions = {
		{},
		{{0, 0}},
		{{0, 0}, {240, 0}, {0, 120}, {240, 120}},
		{{0, 0}},
		{{0, 0}},
		{{0, 0}, {167, 60}, {345, 120}},
	};

	int playingVideoId = 0;
	int refreshedVideoId = -1;

	bool videoDebounce = false;

	vex::timer videoTimePosition;
}

namespace video {
	void keybindVideos() {
		Controller1.ButtonLeft.pressed([]() -> void {
			switchVideoState(-1);
		});
		Controller1.ButtonRight.pressed([]() -> void {
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
			Brain.Screen.clearScreen(color::black);
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
