#pragma once
#include "PixeLINKApi.h"
#include <stdint.h>
#include <set>
#include <math.h>
#include <string>
#include <vector>

#define YUV422 StreamFormat(PIXEL_FORMAT_YUV422,std::string("yuv422"))
#define RGB24 StreamFormat(PIXEL_FORMAT_RGB24_NON_DIB,std::string("8UC3"))
#define MONO8 StreamFormat(PIXEL_FORMAT_MONO8,std::string("mono8"))
#define MONO16 StreamFormat(PIXEL_FORMAT_MONO16,std::string("mono16"))
#define BAYER8 StreamFormat(PIXEL_FORMAT_BAYER8_RGGB,std::string("bayer_rggb8"))


class StreamFormat{
public:
  StreamFormat();
  StreamFormat(int i, std::string s);
  int getPxlFormat() const;
  std::string getRosFormat() const;
  bool operator<(const StreamFormat& other) const{
    int rosResult = rosFormat.compare(other.getRosFormat());
    int pxlResult = pxlFormat - other.getPxlFormat();
    if(rosResult > 0)
      return std::max(rosResult,pxlResult);
    else if(rosResult < 0)
      return std::min(rosResult,pxlResult);
    else if(pxlResult > 0)
      return pxlResult;
    else
      return std::min(rosResult,pxlResult); // 0 if equal, negative otherwise
  };
private:
  std::string rosFormat;
  int pxlFormat;
};


class PxlCamera{
public:
  PxlCamera();
  PxlCamera(HANDLE hCamera);
  void setHandle(HANDLE hCamera);
  HANDLE getHandle();
  double getFrameRate();
  bool setBaseParams();
  bool setFrameRate(float value);
  bool setStreamFormat(float value);
  bool setROI(float* values);
  float getFocalLength();
  void getROI(uint32_t* roi);
  void getPixelReductionRatio(uint32_t* addressing);
  void getImageSize(uint32_t* size);
  uint32_t getWidth();
  uint32_t getHeight();
  uint32_t getBytesPerPixel();
  uint32_t getImageNumBytes();
  std::set<StreamFormat> getStreamFormats();
  std::string getAssocRosFormat(int pxlFormat);
  int getAssocPxlFormat(std::string rosFormat);
  double getCurrentTimeStamp();
  bool getNextFrame(FRAME_DESC * desc, std::vector<uint8_t> frameBuf);
  bool hasRosFormat(std::string rosFormat);
  bool hasPxlFormat(int pxlFormat);
  bool hasFormat(StreamFormat sf);
private:
  HANDLE hCamera;
  std::set<StreamFormat> streamFormats{YUV422,RGB24,MONO8,MONO16,BAYER8};
};
