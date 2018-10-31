#include "pixelink_util.h"

PxlCamera::PxlCamera(){
}
PxlCamera::PxlCamera(HANDLE hCamera){
  this->hCamera = hCamera;
}
void PxlCamera::setHandle(HANDLE hCamera){
  this->hCamera = hCamera;
}
HANDLE PxlCamera::getHandle(){
  return this->hCamera;
}
double PxlCamera::getFrameRate(){
  float value;
  uint32_t flags;
  uint32_t numVals = 1;
  int retCode = PxLGetFeature(hCamera,FEATURE_FRAME_RATE,&flags,&numVals,&value);//defined in PixeLINKTypes.h
  return value;
}
bool PxlCamera::setFrameRate(float value){
  const float valueF = value;
  uint32_t numVals = 1;
  uint32_t retCode = PxLSetFeature(hCamera,FEATURE_FRAME_RATE,FEATURE_FLAG_MANUAL,numVals,&valueF);
  return API_SUCCESS(retCode);
}
bool PxlCamera::setStreamFormat(float value){
  uint32_t numVals = 1;
  const float format = value;
  int retCode = PxLSetFeature(hCamera,FEATURE_PIXEL_FORMAT,FEATURE_FLAG_MANUAL,numVals,&format);
  return API_SUCCESS(retCode);
}
bool PxlCamera::setROI(float* values){
  const float* roi = values;
  uint32_t numVals = 4;
  int retCode = PxLSetFeature(hCamera,FEATURE_ROI,FEATURE_FLAG_MANUAL,numVals,&roi[0]);
  return API_SUCCESS(retCode);
}
void PxlCamera::getROI(uint32_t* roi){
  float values[4];
  uint32_t flags;
  uint32_t numVals = 4;
  int retCode = PxLGetFeature(hCamera,FEATURE_ROI,&flags,&numVals,&values[0]);
  for(int i = 0;i<4;i++){
    roi[i] = (uint32_t)values[i];
  }
}
void PxlCamera::getPixelReductionRatio(uint32_t* addressing){
  float values[4];
  values[2] = 1;
  values[3] = 1;
  uint32_t flags;
  uint32_t numVals = 4;
  int retCode = PxLGetFeature(hCamera,FEATURE_PIXEL_ADDRESSING,&flags,&numVals,&values[0]);
  addressing[0] = (uint32_t)values[2]; // This may be wrong. See https://support.pixelink.com/support/solutions/articles/3000044297-pixel-addressing note about older APIs
  addressing[1] = (uint32_t)values[3];
}
void PxlCamera::getImageSize( uint32_t* size){
  uint32_t roi[4];
  getROI(&roi[0]);
  uint32_t ratio[2];
  getPixelReductionRatio(&ratio[0]);
  size[0] = roi[2]/ratio[0];
  size[1] = roi[3]/ratio[1];
}
uint32_t PxlCamera::getWidth(){
  uint32_t size[2]; 
  getImageSize(size);
  return size[0];
}
uint32_t PxlCamera::getHeight(){
  uint32_t size[2];
  getImageSize(&size[0]);
  return size[1];
}
uint32_t PxlCamera::getBytesPerPixel(){
  float value;
  uint32_t flags;
  uint32_t numVals = 1;
  int retCode = PxLGetFeature(hCamera,FEATURE_PIXEL_FORMAT,&flags,&numVals,&value);
  int retVal = 1;
  // switch((uint32_t)value){
  //   case BAYER8.getPxlFormat():
  //   case MONO8.getPxlFormat():
  //     retVal = 1;
  //     break;
  //   case YUV422.getPxlFormat():
  //   case MONO16.getPxlFormat():
  //     retVal = 2;
  //     break;
  //   case RGB24.getPxlFormat():
  //     retVal = 3;
  //     break;
  return retVal;
}
uint32_t PxlCamera::getImageNumBytes(){
  uint32_t bytesPerPixel = getBytesPerPixel();
  uint32_t size[2]; 
  getImageSize(&size[0]);
  uint32_t numPixels = size[0]*size[1];
  uint32_t numBytes = numPixels*bytesPerPixel;
  return numBytes;
}
std::set<StreamFormat> PxlCamera::getStreamFormats(){
  return streamFormats; 
}
std::string PxlCamera::getAssocRosFormat(int pxlFormat){
  std::set<StreamFormat>::iterator iter;
  for(iter=streamFormats.begin();iter!=streamFormats.end();iter++){
    if(iter->getPxlFormat()==pxlFormat){
      return iter->getRosFormat();
    }
  }
  return std::string("");
}
int PxlCamera::getAssocPxlFormat(std::string rosFormat){
  std::set<StreamFormat>::iterator iter;
  for(iter=streamFormats.begin();iter!=streamFormats.end();iter++){
    if((iter->getRosFormat()).compare(rosFormat)==0){
      return iter->getPxlFormat();
    }
  }
  return -1;
}
bool PxlCamera::hasRosFormat(std::string rosFormat){
  return getAssocPxlFormat(rosFormat) == 0;
}
bool PxlCamera::hasPxlFormat(int pxlFormat){
  return !(getAssocRosFormat(pxlFormat).empty());
}
bool PxlCamera::hasFormat(StreamFormat sf){
  return streamFormats.find(sf)!=streamFormats.end();
}


StreamFormat::StreamFormat(){}
StreamFormat::StreamFormat(int i, std::string s){
  rosFormat = s;
  pxlFormat = i;
}
int StreamFormat::getPxlFormat() const{
  return pxlFormat;
}
std::string StreamFormat::getRosFormat() const{
  return rosFormat;
}