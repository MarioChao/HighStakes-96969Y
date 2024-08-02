#include "Videos/videoInfo.h"
#include "main.h"

/* Video Info */
VideoInfo::VideoInfo() {
    fps = frameSteps = 0;
    savedVideo.clear();
}

VideoInfo::VideoInfo(double videoFps, double videoFrameSteps, std::vector< std::vector< std::vector<int> > > *video) {
    fps = videoFps;
    frameSteps = videoFrameSteps;
    savedVideo = *video;
}

void VideoInfo::saveVideo(std::vector< std::vector< std::vector<int> > > *video) {
    savedVideo = *video;
}

void VideoInfo::loadVideo(std::vector< std::vector< std::vector<int> > > *video, double *frameDelayMs) {
    *video = savedVideo;
    *frameDelayMs = (1.0 / fps) * (1000.0 / 1) * (frameSteps);
}

void VideoInfo::drawFrame(std::vector< std::vector< std::vector<int> > > *video, int x, int y, int width, int height, int frameId) {
    // Validate frame
    if (frameId < 0 || frameId >= (int) video -> size()) {
        return;
    }

    // Get frame
    std::vector< std::vector<int> > frame = (*video)[frameId];

    // Draw frame at position
    int rgb;
    int posX, posY;
    // Loop through columns
    double iStep = (double) frame.size() / height;
    posY = y;
    for (double i = 0; i < (int) frame.size(); i += iStep) {
        // Get column frmae index
        int frameI = (int) i;

        // Loop through rows
        double jStep = (double) frame[frameI].size() / width;
        posX = x;
        for (double j = 0; j < (int) frame[frameI].size(); j += jStep) {
            // Get row frame index
            int frameJ = (int) j;

            // Draw pixel
            rgb = frame[frameI][frameJ];
            if (rgb != -1) {
                Brain.Screen.setPenWidth(1);
                Brain.Screen.setPenColor(color(rgb));
                Brain.Screen.drawPixel(posX, posY);
            }

            // Update
            posX++;
        }
        posY++;
    }
}