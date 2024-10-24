#include "Videos/brainVideos.h"
#include "Videos/VideoInfos/yoruNiKakeru.h"
#include "Videos/VideoInfos/badApple.h"
#include "Videos/VideoInfos/teamLogo.h"
#include "Videos/VideoInfos/ningning.h"
#include "main.h"

namespace {
	void drawFrame(int x, int y, int frameId);

	std::vector< std::vector< std::vector<int> > > video;
	std::vector< std::vector<uint8_t> > videoBuffer;
	std::vector< std::vector< std::vector<bool> > > boolVideo;

	int videoCount = 6;
	double frameDelayMs;
	int frameId;

	double display_width, display_height;

	int videoType = 0; // 0 : video, 1 : bool video

	std::pair<color, color> boolVideoColors;
}

void keybindVideos() {
	Controller1.ButtonLeft.pressed([]() -> void {
		switchVideoState();
	});
}

void brainVideosThread() {
	frameId = 0;
	switchVideoState(0);
	while (true) {
		if (playingVideoId > 0 && frameId >= 0) {
			drawFrame(0, 0, frameId);
			frameId++;
			// Different types of videos
			if (videoType == 0) {
				frameId %= (int) videoBuffer.size();
			} else if (videoType == 1) {
				frameId %= (int) video.size();
			} else if (videoType == 2) {
				frameId %= (int) boolVideo.size();
			}
		}
		task::sleep(frameDelayMs);
	}
}

bool videoDebounce = false;
void switchVideoState(int increment) {
	if (!videoDebounce) {
		videoDebounce = true;

		frameId = -10;
		// Increment video id
		playingVideoId += increment;
		playingVideoId %= (videoCount + 1);
		if (playingVideoId > 0) {
			printf("Playing video %d!\n", playingVideoId);
		}

		// Clear screen
		Brain.Screen.clearScreen(color::black);

		// Switch video
		switch (playingVideoId) {
			case 1:
				teamLogo.loadVideoBuffer(&videoBuffer, &frameDelayMs);
				videoType = !teamLogo.isUsingBuffer();
				break;
			case 2:
				yoruNiKakeru.loadVideo(&video, &frameDelayMs);
				yoruNiKakeru.loadDimensions(&display_width, &display_height);
				videoType = 1;
				break;
			case 3:
				badApple.loadVideo(&boolVideo, &frameDelayMs);
				badApple.loadDimensions(&display_width, &display_height);
				boolVideoColors = badApple.getColors();
				videoType = 2;
				break;
			case 4:
				ningning.loadVideo(&video, &frameDelayMs);
				videoType = !ningning.isUsingBuffer();
				break;
			case 5:
				ningning2.loadVideo(&video, &frameDelayMs);
				videoType = !ningning2.isUsingBuffer();
				break;
			case 6:
				ningning3.loadVideo(&video, &frameDelayMs);
				videoType = !ningning3.isUsingBuffer();
				break;
			case 0:
				video.clear();
				boolVideo.clear();
				break;
		}
		task::sleep(30);
		frameId = 0;
		task::sleep(30);

		videoDebounce = false;
	}
}

namespace {
	void drawFrame(int x, int y, int frameId) {
		// Different types of videos
		if (videoType == 0) {
			VideoInfo::drawBufferFrame(&videoBuffer, x, y, frameId);
		} else if (videoType == 1) {
			VideoInfo::drawFrame(&video, x, y, display_width, display_height, frameId);
		} else if (videoType == 2) {
			BoolVideoInfo::drawFrame(&boolVideo, x, y, display_width, display_height, frameId, boolVideoColors.first, boolVideoColors.second);
		}
	}
}
