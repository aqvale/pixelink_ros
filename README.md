# pixelink_ros
Disclaimer: Provided as is, if you use this and something bad happens, it's not our fault

# To install
Of course, install ROS

Download the [Pixelink drivers](https://storage.googleapis.com/files.pixelink.com/latest/PixeLINKSdk-for-Ubuntu16.04-PC_64-v2.3.tar.gz)

Decompress them inside a folder and follow the installation instructions, except use `LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$PIXELINK_SDK_LIB` when the time comes. 

Ultimately, you will need to have PIXELINK_SDK_INC and PIXELINK_SDK_LIB defined as environment variables. 

## For Ubuntu 18.04
#### In terminal:
sudo vim /etc/apt/sources.list
#### Add in the lines:
deb http://us.archive.ubuntu.com/ubuntu/ xenial main restricted
deb http://us.archive.ubuntu.com/ubuntu/ xenial universe
#### Back in terminal:
sudo apt-get install ffmpeg=7:2.8.6-1ubuntu2

If you're having issues still, try using 

`sudo apt-get purge ffmpeg

 sudo apt autoremove
 
 sudo apt-get install ffmpeg=7:2.8.6-1ubuntu2`

# To run
First, you must be running `roscore` in a terminal

In a new terminal, run `rosrun pixelink_ros pixelink_node`

To view the video, run `rosrun image_view image_view image:=/pixelink/image`

If image_view is not installed, use `sudo apt-get install ros-melodic-image-view`

# Know issues
I don't know if all the services work. Also, the PixeLINK SDK has a lot of capabilities which this ROS API doesn't expose yet. If you want a specific capability, let me know or feel free to make a merge request.
