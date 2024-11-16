#include "Videos/video-info.h"
#include "main.h"

/* Video Info */
VideoInfo::VideoInfo() {
	fps = frameSteps = 0;
	savedVideo.clear();
	savedVideoBuffer.clear();
	display_width = 480;
	display_height = 240;
	use8BitBuffer = false;
}

VideoInfo::VideoInfo(
	double videoFps, double videoFrameSteps, std::vector< std::vector< std::vector<int> > > *video,
	double width, double height
) {
	fps = videoFps;
	frameSteps = videoFrameSteps;
	display_width = width;
	display_height = height;
	saveVideo(video);
}

VideoInfo::VideoInfo(double videoFps, double videoFrameSteps, std::vector< std::vector<uint8_t> > *videoBuffer) {
	fps = videoFps;
	frameSteps = videoFrameSteps;
	saveVideoBuffer(videoBuffer);
}

void VideoInfo::saveVideo(std::vector< std::vector< std::vector<int> > > *video) {
	savedVideo = *video;
	use8BitBuffer = false;
	preProcess();
}

void VideoInfo::saveVideoBuffer(std::vector< std::vector<uint8_t> > *videoBuffer) {
	savedVideoBuffer = *videoBuffer;
	use8BitBuffer = true;
	preProcess();
}

bool VideoInfo::isUsingBuffer() {
	return use8BitBuffer;
}

void VideoInfo::preProcess() {
	// Conversion factors
	frameIdsPerMillisecond = (1.0 / 1000.0) * fps / frameSteps;

	// Frame count
	if (use8BitBuffer) {
		frameCount = (int) savedVideoBuffer.size();
	} else {
		frameCount = (int) savedVideo.size();
	}
}

void VideoInfo::setFrameId(double timePosition_ms) {
	frameId = (int) (timePosition_ms * frameIdsPerMillisecond) % frameCount;
}

void VideoInfo::drawFrame(int x, int y) {
	if (use8BitBuffer) {
		drawFrameBuffer(x, y);
	} else {
		drawFrameRGB(x, y);
	}
}

void VideoInfo::drawFrameRGB(int x, int y) {
	// Validate frame
	if (frameId < 0 || frameId >= frameCount) {
		return;
	}

	// Get frame
	std::vector< std::vector<int> > frame = savedVideo[frameId];

	/* Draw frame at position */

	// Pixel variables
	int pixel_rgb;
	int pixel_x, pixel_y;

	// For each horizontal row in the frame
	double iStep = (double) frame.size() / display_height;
	pixel_y = y;
	for (double i = 0; i < (int) frame.size(); i += iStep) {
		// Get integer row index
		int frameI = (int) i;

		// For each vertical column in the row
		double jStep = (double) frame[frameI].size() / display_width;
		pixel_x = x;
		for (double j = 0; j < (int) frame[frameI].size(); j += jStep) {
			// Get integer column index
			int frameJ = (int) j;

			// Draw pixel
			pixel_rgb = frame[frameI][frameJ];
			if (pixel_rgb != -1) {
				Brain.Screen.setPenWidth(1);
				Brain.Screen.setPenColor(vex::color(pixel_rgb));
				Brain.Screen.drawPixel(pixel_x, pixel_y);
			}

			// Update
			pixel_x++;
		}
		pixel_y++;
	}
}

void VideoInfo::drawFrameBuffer(int x, int y) {
	// Validate frame
	if (frameId < 0 || frameId >= frameCount) {
		return;
	}

	// Get frame
	uint8_t *frame = &savedVideoBuffer[frameId][0];
	int frameSize = (int) savedVideoBuffer[frameId].size();

	// Draw frame at position
	Brain.Screen.drawImageFromBuffer(frame, x, y, frameSize);
}
