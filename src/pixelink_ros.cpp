#include <vector.h>
#include <stdio.h>
#include <time.h>

//PixeLINK stuff
#include "PixeLINKApi.h"
#include "pixelink_util.h"//My own stuff on top of the api

#include <ros/ros.h>
#include <image_transport/image_transport>
#include "pixelink_ros/setFrameRate.srv"
#include "pixelink_ros/setROI.srv"
#include "pixelink_ros/setOutputFormat.srv"
#include "pixelink_ros/setStreamFormat.srv"

//Globals here
float fps;
int width, height, numBytes,count;
char* outputFormat = "8UC3";
uint32_t streamFormat = PIXEL_FORMAT_YUV422;
HANDLE hCamera;

//Insert services here
bool callbackFrameRate(pixelink_ros::setFrameRate::Request& req, pixelink_ros::setFrameRate::Response& res){
  // Check if requested framerate is reasonable
  if(req.frameRate < 35 && req.frameRate>0){
    if(setFrameRate(hCamera,req.frameRate)){
      fps = req.frameRate;
      res.success = true;
      return true;
    }
  }
  res.success=false;
  return false;
}

bool callbackROI(pixelink_ros::setROI::Request& req, pixelink_ros::setROI::Response& res){
  // Check roi vs max width, max height, min x and y offsets (ROI fits in possible window)
  if(req.width+req.xOff<xMax && req.height+req.yOff<yMax && req.xOff>=0 && req.yOff>=0){
    width = req.width;
    height = req.height;
    xOff = req.xOff;
    yOff = req.yOff;
    res.success = true;
    return true;
  }
  res.success=false;
  return false;
}

bool callbackStreamFormat(pixelink_ros::setStreamFormat::Request& req, pixelink_ros::setStreamFormat::Response& res){
  //TODO: Define mappings in pixelink_util.h
  //line 250 of PixeLINKTypes.h has the correct names
  //Perhaps use the same strings as given by sensor messages image encodings definitions
  if(valid){
    if(setStreamFormat(hCamera,formatAsInteger)){
      streamFormat = formatAsInteger;
      res.success = true;
      return true;
    }
  }
  res.success = false;
  return false;
}

bool callbackOutputFormat(pixelink_ros::setOutputFormat::Request& req, pixelink_ros::setOutputFormat::Response& res){
  // TODO: Check if req.format is legal
  res.success = legal;
  if(legal){
    encoding = req.format;//TODO: GET THE CONVERSION TO A CHAR ARRAY RIGHT
    return true;
  }
  return false;
}
// Set fps (PxLSetStreamRate)
/*
  uint64_t flags;
  float value;
  retCode = PxLGetFeature(hCamera,FEATURE_IRIS,&flags,1,&value);//defined in PixeLINKTypes.h
  PxLSetFeature(hCamera,FEATURE_IRIS,FEATURE_FLAG_MANUAL,1,&value);
*/
// Set size of image
// Set other various parameters

//Main code
int main(int argc, char** argv){
  ros::init(argc,argv,"pixelink")
  ros::NodeHandle nh = new ros::NodeHandle();
  

  //Initialize Camera
  uint32_t nCameras = 0;
  uint32_t retCode = 0;
  retCode = PxLGetNumberCameras(NULL, &nCameras);
  
  if(!API_SUCCESS(retCode) || nCameras<1){
    prtinf(" Error: No cameras found");
    return -1;
  }

  retCode = PxLInitialize(0, &hCamera);
  if(!API_SUCCESS(retCode)){
    printf(" Error: Camera initialization failed");
    return -1;
  }

  //Get camera parameters
  fps = getFrameRate(hCamera);
  width = getImageWidth(hCamera);
  height = getImageHeight(hCamera);
  numBytes = getImageNumBytes(hCamera);
  std::vector<uint8_t> frameBuf(numBytes);
  uint8_t* imageRGB[3*width*height];

  ROS_INFO("Camera Initialized");

  //Initialize ROS image transport stuff
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("image")
  ros::ServiceClient frameRateClient = nh.serviceClient<pixelink_ros::setFrameRate>("/pixelink/setFrameRate");
  ros::ServiceClient rOIClient = nh.serviceClient<pixelink_ros::setROI>("/pixelink/setROI");
  ros::ServiceClient streamFormatClient = nh.serviceClient<pixelink_ros::setStreamFormat>("/pixelink/setStreamFormat");
  ros::ServiceClient outputFormatClient = nh.serviceClient<pixelink_ros::setOutputFormat>("/pixelink/setOutputFormat");

  //Begin looping
  ros::Rate rate(fps);
  while(ros::ok()){
    // Get next frame
    FRAME_DESC* desc;
    desc.uSize = sizeof(desc);
    retCode = PxLGetNextFrame(hCamera,frameBuf.size(),&frameBuf[0],desc);

    // convert to image_transport
    uint32_t bytesToWrite = 0;
    retCode = PxLFormatImage(&frameBuf[0], &desc, IMAGE_FORMAT_RAW_RGB24, &imageRGB[0], &bytesToWrite);
    sensor_msgs::ImagePtr msg;
    msg->width = width;
    msg->height = height;
    msg->step = width*3;//3 for rgba
    memcpy(&(msg.data[0]),&imageRGB[0],3*width*height);
    msg->encoding = encoding;// note, this can also be yuv422 or bayer: http://docs.ros.org/jade/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html
    msg->is_bigendian = 0;
    std_msgs::Header header;
    header.seq = count++;
    header.timestamp = ros::Time::now();
    header.frame_id = "0";
    msg->header = header;
    // Send frame
    pub.publish(msg);


    rate.sleep();
  }

}