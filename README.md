# pixelink_ros
Disclaimer: Provided as is, if you use this and something bad happens, it's not our fault

# To install
Of course, install ROS

Download the [Pixelink drivers](https://storage.googleapis.com/files.pixelink.com/latest/PixeLINKSdk-for-Ubuntu16.04-PC_64-v2.3.tar.gz)

Decompress them inside a folder and follow the installation instructions, except use `LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$PIXELINK_SDK_LIB` when the time comes.

Ultimately, you will need to have PIXELINK_SDK_INC and PIXELINK_SDK_LIB defined as environment variables. 
