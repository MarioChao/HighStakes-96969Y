#include "Videos/boolVideoInfo.h"
#include "main.h"

/* Boolean Video Info */
BoolVideoInfo::BoolVideoInfo() {
    fps = frameSteps = 0;
    savedVideo.clear();
}

BoolVideoInfo::BoolVideoInfo(double videoFps, double videoFrameSteps, std::vector< std::vector< std::vector<bool> > > *video, color onColor, color offColor) {
    fps = videoFps;
    frameSteps = videoFrameSteps;
    savedVideo = *video;
    this -> onColor = onColor;
    this -> offColor = offColor;
}

void BoolVideoInfo::saveVideo(std::vector< std::vector< std::vector<bool> > > *video) {
    savedVideo = *video;
}

void BoolVideoInfo::loadVideo(std::vector< std::vector< std::vector<bool> > > *video, double *frameDelayMs) {
    *video = savedVideo;
    *frameDelayMs = (1.0 / fps) * (1000.0 / 1) * (frameSteps);
}

std::pair<color, color> BoolVideoInfo::getColors() {
    // printf("%d, %d\n", onColor.rgb(), offColor.rgb());
    return std::make_pair(onColor, offColor);
}

void BoolVideoInfo::drawFrame(std::vector< std::vector< std::vector<bool> > > *video, int x, int y, int width, int height, int frameId, color onColor, color offColor) {
    // Validate frame
    if (frameId < 0 || frameId >= (int) video -> size()) {
        return;
    }

    // Get frame
    std::vector< std::vector<bool> > frame = (*video)[frameId];

    // Draw frame at position
    bool isColorOn;
    int posX, posY;
    // Loop through columns
    double iStep = (double) frame.size() / height;
    posY = y;
    for (double i = 0; i < (int) frame.size(); i += iStep) {
        // Get column frame index
        int frameI = (int) i;

        // Loop through rows
        double jStep = (double) frame[frameI].size() / width;
        posX = x;
        for (double j = 0; j < (int) frame[frameI].size(); j += jStep) {
            // Get row frame index
            int frameJ = (int) j;

            // Draw pixel
            isColorOn = frame[frameI][frameJ];
            Brain.Screen.setPenWidth(1);
            if (isColorOn) {
                Brain.Screen.setPenColor(onColor);
            } else {
                Brain.Screen.setPenColor(offColor);
            }
            Brain.Screen.drawPixel(posX, posY);
            
            // Update
            posX++;
        }
        posY++;
    }
}