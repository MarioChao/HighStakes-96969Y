#include "Videos/brainVideos.h"
#include "Videos/VideoInfos/yoruNiKakeru.h"
#include "Videos/VideoInfos/badApple.h"
#include "Videos/VideoInfos/teamLogo.h"
#include "main.h"

namespace {
    void drawFrame(int x, int y, int width, int height, int frameId);

    std::vector< std::vector< std::vector<int> > > video;
    std::vector< std::vector< std::vector<bool> > > boolVideo;

    int videoCount = 3;
    double frameDelayMs;
    int frameId;

    int videoType = 0; // 0 : video, 1 : bool video

    std::pair<color, color> boolVideoColors;
}

void keybindVideos() {
    Controller1.ButtonLeft.pressed([] () -> void {
        switchVideoState();
    });
}

void brainVideosThread() {
    frameId = 0;
    switchVideoState(0);
    while (true) {
        if (playingVideoId > 0 && frameId >= 0) {
            drawFrame(0, 0, 480, 240, frameId);
            frameId++;
            if (videoType == 0) {
                frameId %= (int) video.size();
            } else if (videoType == 1) {
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
            // printf("Playing video %d!\n", playingVideoId);
        }

        // Switch video
        switch (playingVideoId) {
            case 1:
                teamLogo.loadVideo(&video, &frameDelayMs);
                videoType = 0;
                break;
            case 2:
                yoruNiKakeru.loadVideo(&video, &frameDelayMs);
                videoType = 0;
                break;
            case 3:
                badApple.loadVideo(&boolVideo, &frameDelayMs);
                boolVideoColors = badApple.getColors();
                videoType = 1;
                break;
            case 0:
                video.clear();
                boolVideo.clear();
        }
        task::sleep(30);
        frameId = 0;
        task::sleep(30);

        videoDebounce = false;
    }
}

namespace {
    void drawFrame(int x, int y, int width, int height, int frameId) {
        // Two types of videos
        if (videoType == 0) {
            VideoInfo::drawFrame(&video, x, y, width, height, frameId);
        } else if (videoType == 1) {
            BoolVideoInfo::drawFrame(&boolVideo, x, y, width, height, frameId, boolVideoColors.first, boolVideoColors.second);
        }
    }
}