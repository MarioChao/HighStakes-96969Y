#include "Videos/videoInfo.h"
#include "main.h"

/* Video Info */
VideoInfo::VideoInfo() {
	fps = frameSteps = 0;
	savedVideo.clear();
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
	savedVideo = *video;
	display_width = width;
	display_height = height;
	use8BitBuffer = false;
}

VideoInfo::VideoInfo(double videoFps, double videoFrameSteps, std::vector< std::vector<uint8_t> > *videoBuffer) {
	fps = videoFps;
	frameSteps = videoFrameSteps;
	savedVideoBuffer = *videoBuffer;
	use8BitBuffer = true;
}

void VideoInfo::saveVideo(std::vector< std::vector< std::vector<int> > > *video) {
	savedVideo = *video;
}

void VideoInfo::loadVideo(std::vector< std::vector< std::vector<int> > > *video, double *frameDelayMs) {
	*video = savedVideo;
	*frameDelayMs = (1.0 / fps) * (1000.0 / 1) * (frameSteps);
}

void VideoInfo::loadDimensions(double *width, double *height) {
	*width = display_width;
	*height = display_height;
}

bool VideoInfo::isUsingBuffer() {
	return use8BitBuffer;
}

void VideoInfo::saveVideoBuffer(std::vector< std::vector<uint8_t> > *videoBuffer) {
	savedVideoBuffer = *videoBuffer;
	use8BitBuffer = true;
}

void VideoInfo::loadVideoBuffer(std::vector< std::vector<uint8_t> > *videoBuffer, double *frameDelayMs) {
	*videoBuffer = savedVideoBuffer;
	*frameDelayMs = (1.0 / fps) * (1000.0 / 1) * (frameSteps);
}

void VideoInfo::drawFrame(std::vector< std::vector< std::vector<int> > > *video, int x, int y, int width, int height, int frameId) {
	// Validate frame
	if (frameId < 0 || frameId >= (int) video->size()) {
		return;
	}

	// Get frame
	std::vector< std::vector<int> > frame = (*video)[frameId];
	// printf("Frame: %d\n", frameId);

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

void VideoInfo::drawBufferFrame(std::vector< std::vector<uint8_t> > *videoBuffer, int x, int y, int frameId) {
	// Validate frame
	if (frameId < 0 || frameId >= (int) videoBuffer->size()) {
		return;
	}

	// Get frame
	uint8_t *frame = &(*videoBuffer)[frameId][0];
	int frameSize = (int) (*videoBuffer)[frameId].size();
	// printf("%d %d %d\n", frame[0], frame[1], frame[2]);
	// printf("Size: %d\n", frameSize);

	// Draw frame at position
	Brain.Screen.drawImageFromBuffer(frame, x, y, frameSize);
}
