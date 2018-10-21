#include "pixelink_util.h"

double getFrameRate(HANDLE hCamera){
  double value;
  uint64_t flags;
  uint32_t retCode = PxLGetFeature(hCamera,FEATURE_FRAME_RATE,&flags,1,&value);//defined in PixeLINKTypes.h
  return value;
}
uint32_t* getROI(HANDLE hCamera){
  float values[4];
  uint32_t roi[2];
  uint64_t flags;
  uint32_t retCode = PxLGetFeature(hCamera,FEATURE_ROI,4,&values[0]);
  roi[0] = (uint32_t)values[0];
  roi[1] = (uint32_t)values[1];
  return roi;
}
uint32_t* getPixelReductionRatio(HANDLE hCamera){
  float values[4];
  values[2] = 1;
  values[3] = 1;
  uint32_t addressing[2];
  uint64_t flags;
  uint32_t retCode = PxLGetFeature(hCamera,FEATURE_PIXEL_ADDRESSING,4,&values[0]);
  addressing[0] = (uint32_t)values[2]; // This may be wrong. See https://support.pixelink.com/support/solutions/articles/3000044297-pixel-addressing note about older APIs
  addressing[1] = (uint32_t)values[3];
  return addressing;
}
uint32_t* getImageSize(HANDLE hCamera){
  uint32_t size[2];
  uint32_t* roi = getROI(hCamera);
  uint32_t* ratio = getPixelReductionRatio(hCamera);
  size[0] = roi[0]/ratio[0];
  size[1] = roi[1]/ratio[1];
  return size;
}
uint32_t getWidth(HANDLE hCamera){
  uint32_t* size = getImageSize(hCamera);
  return size[0];
}
uint32_t getHeight(HANDLE hCamera){
  uint32_t* size = getImageSize(hCamera);
  return size[1];
}
uint32_t getBytesPerPixel(HANDLE hCamera){
  float value;
  uint64_t flags;
  uint32_t retCode = PxLGetFeature(hCamera,FEATURE_PIXEL_FORMAT,&flags,1,&value);
  return (uint32_t)value;
}
uint32_t getImageNumBytes(HANDLE hCamera){
  uint32_t bytesPerPixel = getBytesPerPixel(hCamera);
  uint32_t* size = getImageSize(hCamera);
  uint32_t numPixels = size[0]*size[1];
  uint32_t numBytes = numPixels*bytesPerPixel;
  return numBytes;
}