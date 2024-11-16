#include "Videos/video-objects.h"

/*
480 x 240
round RGB: 1

Start frame: 0
Frame count: 1
Frame step: 10
FPS: 30

with https://imageresizer.com/png-compressor/
or https://compresspng.com/
*/

namespace {

std::vector< std::vector< std::vector<int> > > video = {};

std::vector< std::vector<uint8_t> > videoBuffer = {};

}

VideoInfo _exampleVideo(30, 10, &::video, 480, 240);
// VideoInfo _exampleVideo(30, 10, &::videoBuffer);
