#pragma once

#include <vector>
#include <main.h>

class BoolVideoInfo {
    public:
        BoolVideoInfo();
        BoolVideoInfo(double fps, double frameSteps, std::vector< std::vector< std::vector<bool> > > *video, color onColor, color offColor);
        void saveVideo(std::vector< std::vector< std::vector<bool> > > *video);
        void loadVideo(std::vector< std::vector< std::vector<bool> > > *video, double *frameDelayMs);
        std::pair<color, color> getColors();

        static void drawFrame(std::vector< std::vector< std::vector<bool> > > *video, int x, int y, int width, int height, int frameId, color onColor, color offColor);
    private:
        double fps;
        double frameSteps;
        std::vector< std::vector< std::vector<bool> > > savedVideo;
        color onColor, offColor;
};