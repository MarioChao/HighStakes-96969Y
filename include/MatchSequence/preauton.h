#pragma once

namespace preauton {
void controllerThread();
void run();
void calibrateIMU();
bool isFinished();
bool isBufferFinished();
}
