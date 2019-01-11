cmake_minimum_required(VERSION 3.0)
project(pixelink_ros)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -g -Wall)

find_package(catkin REQUIRED COMPONENTS
  image_transport
  std_msgs
  roscpp
  message_generation
)

set(PXL_LIB_PATH $ENV{PIXELINK_SDK_LIB})
set(PXL_INC $ENV{PIXELINK_SDK_INC})

find_library(PXL_LIB PATHS ${PXL_LIB_PATH} NAMES PxLApi)

add_service_files(DIRECTORY srv FILES
  setFrameRate.srv
  setOutputFormat.srv
  setROI.srv
  setStreamFormat.srv
  getROI.srv
  getFocus.srv
)
  
generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS
  std_msgs
  image_transport
  roscpp
  message_runtime
)

#execute_process(
#  COMMAND "cp " ${PROJECT_SOURCE_DIR}"/third_party/libPxLApi.so /usr/lib/libPxLApi.so"}
#)

#########
# BUILD #
#########
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  ${PXL_INC}
)
add_definitions(-DPIXELINK_LINUX)

add_executable(pixelink_node
               src/pixelink_ros.cpp
               src/pixelink_util.cpp)

set(EXEC_LIST pixelink_node)
foreach(EXEC_FILE ${EXEC_LIST})
  target_link_libraries(${EXEC_FILE} ${catkin_LIBRARIES} ${PXL_LIB})
  add_dependencies(${EXEC_FILE} ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
  install(TARGETS ${EXEC_FILE}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endforeach(EXEC_FILE ${EXEC_LIST})
#install(DIRECTORY launch/
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#  PATTERN ".svn" EXCLUDE
#)
