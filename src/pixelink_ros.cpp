#include <vector>
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <cmath>

#include <ros/ros.h>
//#include "PixeLINKApi.h"
#include "pixelink_util.h"
#include <image_transport/image_transport>

//Globals here
float fps;
int width, height, numBytes,count;

//Insert services here
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
  HANDLE hCamera;
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
    msg->encoding = "8UC3";// note, this can also be yuv422 or bayer: http://docs.ros.org/jade/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html
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
