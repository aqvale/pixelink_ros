#pragma once
#include "PixeLINKApi.h"
#include <stdint.h>

double getFrameRate(void* hCamera);
uint32_t* getROI(void* hCamera);
uint32_t* getPixelReductionRatio(void* hCamera);
uint32_t* getImageSize(void* hCamera);
uint32_t getWidth(void* hCamera);
uint32_t getHeight(void* hCamera);
uint32_t getBytesPerPixel(void* hCamera);
uint32_t getImageNumBytes(void* hCamera);
