#include <vector>
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <cmath>
#include <stdio.h>

//PixeLINK stuff
#include "PixeLINKApi.h"
#include "pixelink_util.h"//My own stuff on top of the api

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include "pixelink_ros/setFrameRate.h"
#include "pixelink_ros/setROI.h"
#include "pixelink_ros/setOutputFormat.h"
#include "pixelink_ros/setStreamFormat.h"
#include "pixelink_ros/getFocus.h"
#include "pixelink_ros/getROI.h"

//Globals here
float fps;
int width, height, numBytes, count, xMax = 2448, yMax = 2048, xOff = 0, yOff = 0;
StreamFormat outputFormat = BAYER8;
StreamFormat cameraFormat = BAYER8;
HANDLE hCamera;
PxlCamera cam;

//Insert services here
bool callbackFrameRate(pixelink_ros::setFrameRate::Request& req, pixelink_ros::setFrameRate::Response& res){
  // Check if requested framerate is reasonable
  if(req.frameRate < 35 && req.frameRate>0){
    if(cam.setFrameRate(req.frameRate)){
      fps = req.frameRate;
      res.success = true;
       return true;
    }
  }
  res.success=false;
  return false;
}

bool callbackSetROI(pixelink_ros::setROI::Request& req, pixelink_ros::setROI::Response& res){
  // Check roi vs max width, max height, min x and y offsets (ROI fits in possible window)
  if(req.width+req.xOff<xMax && req.height+req.yOff<yMax && req.xOff>=0 && req.yOff>=0){
    float roi[4] = {(float)xOff,(float)yOff,(float)width,(float)height};
    if(cam.setROI(roi)){
      width = req.width;
      height = req.height;
      xOff = req.xOff;
      yOff = req.yOff;
      res.success = true;
      return true;
    }
  }
  res.success=false;
  return false;
}

bool callbackStreamFormat(pixelink_ros::setStreamFormat::Request& req, pixelink_ros::setStreamFormat::Response& res){
  if(cam.hasRosFormat(req.format)){ // Setting stream format with a string
    if(cam.setStreamFormat(cam.getAssocPxlFormat(req.format))){
      cameraFormat = StreamFormat(cam.getAssocPxlFormat(req.format),req.format);
      res.success = true;
      return true;
    }
  }
  res.success = false;
  return false;
}

bool callbackOutputFormat(pixelink_ros::setOutputFormat::Request& req, pixelink_ros::setOutputFormat::Response& res){
  // TODO: Check if req.format is legal
  bool legal = cam.hasRosFormat(req.format);
  res.success = legal;
  if(legal){
    outputFormat = StreamFormat(cam.getAssocPxlFormat(req.format),req.format);
    return true;
  }
  return false;
}

bool callbackFocus(pixelink_ros::getFocus::Request& req, pixelink_ros::getFocus::Response& res){
  res.focus = cam.getFocalLength();
  return true;
}

bool callbackGetROI(pixelink_ros::getROI::Request& req, pixelink_ros::getROI::Response& res){
  uint32_t roi[4];
  cam.getROI(&roi[0]);
  for(int i=0;i<2;i++){
    res.size[i] = roi[2+i];
    res.offset[i] = roi[i];
  }
  return true;
}

//Main code
int main(int argc, char** argv){
  ros::init(argc,argv,"pixelink");
  ros::NodeHandle nh;
  ROS_INFO("Initialized ROS");

  //Initialize Camera
  uint32_t nCameras = 0;
  std::vector<uint32_t> connectedList;
  uint32_t retCode = 0;
  
  retCode = PxLGetNumberCameras(NULL, &nCameras);
  if(!API_SUCCESS(retCode) || nCameras<1){
    printf(" Error: No cameras found");
    return -1;
  }
  // These two lines are necessary b/c PxLGetNumberCameras CHECKS THE VECTOR SIZE BEFORE FILLING ID'S
  connectedList.resize(nCameras);
  retCode = PxLGetNumberCameras(&connectedList[0],&nCameras);
  retCode = PxLInitialize(connectedList[0], &hCamera);
  if(!API_SUCCESS(retCode)){
    printf(" Error: Camera initialization failed with code %x\n",retCode);
    return -1;
  }
  retCode = PxLSetStreamState (hCamera, STOP_STREAM);

  cam = PxlCamera(hCamera);
  cam.setBaseParams();
  //Get camera parameters
  fps = cam.getFrameRate();
  width = cam.getWidth();
  height = cam.getHeight();
  numBytes = cam.getImageNumBytes();
  std::vector<uint8_t> frameBuf(numBytes);
  ROS_INFO("Camera Initialized");
  //uint8_t* imageRGB[3*width*height];
  std::vector<uint8_t> imageRGB;
  retCode = PxLSetStreamState (hCamera, START_STREAM);

  ROS_INFO("Camera Initialized");
  ROS_INFO_STREAM("Properties: FPS = "<<fps<<"\n\t\twidth = "<<width<<"\n\t\theight="<<height);

  //Initialize ROS image transport stuff
  //image_transport::ImageTransport it(nh);
  //image_transport::Publisher pub = it.advertise("image",2);
  ros::Publisher pub = nh.advertise<sensor_msgs::Image>("/pixelink/image",2);
  ros::ServiceServer frameRateServer = nh.advertiseService("/pixelink/setFrameRate",callbackFrameRate);
  ros::ServiceServer setROIServer = nh.advertiseService("/pixelink/setROI",callbackSetROI);
  ros::ServiceServer streamFormatServer = nh.advertiseService("/pixelink/setStreamFormat",callbackStreamFormat);
  ros::ServiceServer outputFormatServer = nh.advertiseService("/pixelink/setOutputFormat",callbackOutputFormat);
  ros::ServiceServer focusServer = nh.advertiseService("/pixelink/getFocus",callbackFocus);
  ros::ServiceServer getROIServer = nh.advertiseService("/pixelink/getROI",callbackGetROI);

  // ros::ServiceClient focusClient = nh.serviceClient<pixelink_ros::getFocus>("/pixelink/getFocus");
  // pixelink_ros::getFocus srv;
  // focusClient.call(srv);
  // ROS_INFO_STREAM("Focus is " << srv.response.focus);
  ROS_INFO("ROS Image Transport publisher/services set up successfully");
  
  //Begin looping
  ros::Rate rate(fps);
  while(ros::ok()){
    // Get next frame
    FRAME_DESC desc;
    desc.uSize = sizeof(FRAME_DESC);
    retCode = PxLGetNextFrame(hCamera,frameBuf.size(),&frameBuf[0],&desc);
    if(!API_SUCCESS(retCode)){
      printf(" Error: Failed to obtain frame %x\n",retCode);
      return -1;
    }

    // convert to image_transport
    // uint32_t bytesToWrite = 0;
    // retCode = PxLFormatImage(&frameBuf[0], &desc, outputFormat.getPxlFormat(), NULL, &bytesToWrite);
    // imageRGB.resize(bytesToWrite);
    // retCode = PxLFormatImage(&frameBuf[0], &desc, outputFormat.getPxlFormat(), &imageRGB[0], &bytesToWrite);
    // if(!API_SUCCESS(retCode)){
    //   printf(" Error: Failed to convert frame %x\n",retCode);
    //   return -1;
    // }
    sensor_msgs::Image msg;
    msg.width = width;
    msg.height = height;
    msg.step = frameBuf.size()/height;//3 for rgba
    msg.data.resize(frameBuf.size());
    memcpy(&(msg.data[0]),&frameBuf[0],frameBuf.size());
    msg.encoding = outputFormat.getRosFormat();// note, this can also be yuv422 or bayer: http://docs.ros.org/jade/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html
    msg.is_bigendian = 0;
    std_msgs::Header header;
    header.seq = count++;
    header.stamp = ros::Time::now();
    header.frame_id = "0";
    msg.header = header;
    // Send frame
    pub.publish(msg);
    ros::spinOnce();

    rate.sleep();
  }

}
