#pragma once

#include <vector>
#include <main.h>

class BoolVideoInfo {
public:
	BoolVideoInfo();
	BoolVideoInfo(
		double fps, double frameSteps, std::vector< std::vector< std::vector<bool> > > *video, color onColor, color offColor,
		double display_width = 480, double display_height = 240
	);
	void saveVideo(std::vector< std::vector< std::vector<bool> > > *video);
	void loadVideo(std::vector< std::vector< std::vector<bool> > > *video, double *frameDelayMs);
	void loadDimensions(double *width, double *height);
	std::pair<color, color> getColors();

	static void drawFrame(std::vector< std::vector< std::vector<bool> > > *video, int x, int y, int width, int height, int frameId, color onColor, color offColor);
private:
	double fps;
	double frameSteps;
	std::vector< std::vector< std::vector<bool> > > savedVideo;
	color onColor, offColor;
	int display_width, display_height;
};
