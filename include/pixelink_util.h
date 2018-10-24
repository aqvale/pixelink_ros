//#include "PixeLINKApi.h"
#include <cmath>
#include <math.h>
#include <unistd.h>
#include <cstdlib>

double getFrameRate(HANDLE hCamera);
uint32_t* getROI(HANDLE hCamera);
uint32_t* getPixelReductionRatio(HANDLE hCamera);
uint32_t* getImageSize(HANDLE hCamera);
uint32_t getWidth(HANDLE hCamera);
uint32_t getHeight(HANDLE hCamera);
uint32_t getBytesPerPixel(HANDLE hCamera);
uint32_t getImageNumBytes(HANDLE hCamera);
