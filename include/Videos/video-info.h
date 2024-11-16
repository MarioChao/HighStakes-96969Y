#pragma once

#include <cstdint>
#include <vector>

class VideoInfo {
public:
	VideoInfo();
	VideoInfo(
		double fps, double frameSteps, std::vector< std::vector< std::vector<int> > > *video,
		double width = 480, double height = 240
	);
	VideoInfo(double fps, double frameSteps, std::vector< std::vector<uint8_t> > *videoBuffer);
	void saveVideo(std::vector< std::vector< std::vector<int> > > *video);
	
	void saveVideoBuffer(std::vector< std::vector<uint8_t> > *videoBuffer);
	bool isUsingBuffer();

	void preProcess();

	void setFrameId(double timePosition_ms);

	void drawFrame(int x = 0, int y = 0);
	void drawFrameRGB(int x = 0, int y = 0);
	void drawFrameBuffer(int x = 0, int y = 0);

private:
	// Configurations
	double fps;
	double frameSteps;
	std::vector< std::vector< std::vector<int> > > savedVideo;
	std::vector< std::vector<uint8_t> > savedVideoBuffer;
	int display_width, display_height;
	bool use8BitBuffer;

	// Drawing
	double frameIdsPerMillisecond;
	int frameCount;
	int frameId;
};
