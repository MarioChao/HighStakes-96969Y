#pragma once

#include <vector>

class VideoInfo {
    public:
        VideoInfo();
        VideoInfo(double fps, double frameSteps, std::vector< std::vector< std::vector<int> > > *video);
        void saveVideo(std::vector< std::vector< std::vector<int> > > *video);
        void loadVideo(std::vector< std::vector< std::vector<int> > > *video, double *frameDelayMs);

        static void drawFrame(std::vector< std::vector< std::vector<int> > > *video, int x, int y, int width, int height, int frameId);
    private:
        double fps;
        double frameSteps;
        std::vector< std::vector< std::vector<int> > > savedVideo;
};