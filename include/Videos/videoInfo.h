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
	void loadVideo(std::vector< std::vector< std::vector<int> > > *video, double *frameDelayMs);
	void loadDimensions(double *width, double *height);
	
	bool isUsingBuffer();
	void saveVideoBuffer(std::vector< std::vector<uint8_t> > *videoBuffer);
	void loadVideoBuffer(std::vector< std::vector<uint8_t> > *videoBuffer, double *frameDelayMs);

	static void drawFrame(std::vector< std::vector< std::vector<int> > > *video, int x, int y, int width, int height, int frameId);
	static void drawBufferFrame(std::vector< std::vector<uint8_t> > *videoBuffer, int x, int y, int frameId);
private:
	double fps;
	double frameSteps;
	std::vector< std::vector< std::vector<int> > > savedVideo;
	std::vector< std::vector<uint8_t> > savedVideoBuffer;
	int display_width, display_height;
	bool use8BitBuffer;
};
