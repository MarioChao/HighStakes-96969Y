#include "Videos/brainVideos.h"
#include "Videos/boolVideoInfo.h"
#include "Videos/videoInfo.h"
#include "Videos/VideoInfos/yoruNiKakeru.h"
#include "Videos/VideoInfos/badApple.h"
#include "Videos/VideoInfos/teamLogo.h"
#include "Videos/VideoInfos/ningning.h"
#include "main.h"

namespace {
	void handleLoadVideo(VideoInfo &videoInfo);
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

		// Switch video
		switch (playingVideoId) {
			case 1:
				// teamLogo.loadVideo(&video, &frameDelayMs);
				// teamLogo.loadDimensions(&display_width, &display_height);
				// videoType = !teamLogo.isUsingBuffer();
				handleLoadVideo(teamLogo);
				break;
			case 2:
				handleLoadVideo(yoruNiKakeru);
				break;
			case 3:
				handleLoadVideo(badApple);
				break;
			case 4:
				handleLoadVideo(ningning);
				break;
			case 5:
				handleLoadVideo(ningning2);
				break;
			case 6:
				handleLoadVideo(ningning3);
				break;
			case 0:
				video.clear();
				boolVideo.clear();
				break;
		}

		// Refresh screen
		task::sleep(30);
		Brain.Screen.clearScreen(color::black);
		frameId = 0;
		task::sleep(30);

		videoDebounce = false;
	}
}

namespace {
	void handleLoadVideo(VideoInfo &videoInfo) {
		if (videoInfo.isUsingBuffer()) {
			videoInfo.loadVideoBuffer(&videoBuffer, &frameDelayMs);
			videoType = 0;
		} else {
			videoInfo.loadVideo(&video, &frameDelayMs);
			videoInfo.loadDimensions(&display_width, &display_height);
			videoType = 1;
		}
	}

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
