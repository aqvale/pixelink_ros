#pragma once
#include "PixeLINKApi.h"
#include <stdint.h>

class PxlCamera{
public:
  PxlCamera();
  PxlCamera(void* hCamera);
  ~PxlCamera();
  void setHandle(void* hCamera);
  void* getHandle();
  double getFrameRate();
  bool setFrameRate(float value);
  bool setStreamFormat(float value);
  bool setROI(float* values);
  void getROI(uint32_t* roi);
  void getPixelReductionRatio(uint32_t* addressing);
  void getImageSize(uint32_t* size);
  uint32_t getWidth();
  uint32_t getHeight();
  uint32_t getBytesPerPixel();
  uint32_t getImageNumBytes();
private:
  void* hCamera;
};