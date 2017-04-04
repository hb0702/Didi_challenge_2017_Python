cmake_minimum_required(VERSION 2.8.3)
project(kor_didi_pkg)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  rosbag
  cv_bridge
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
# catkin_python_setup()

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

catkin_package(
  INCLUDE_DIRS
    include
#  LIBRARIES kor_didi_pkg
  CATKIN_DEPENDS
    roscpp
    std_msgs
    rosbag
    cv_bridge
  DEPENDS
    system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/kor_didi_pkg.cpp
# )

#This will generate message header file before building the target
#add_dependencies(image_processor ???message_file_header_cpp??)
#add_dependencies(kf_localizer ???message_file_header_cpp??)

#This will create executables of the nodes
add_executable(bag_publisher src/bag_publisher.cpp)
add_executable(image_processor src/image_processor.cpp)
add_executable(kf_localizer src/kf_localizer.cpp)

#This will link executables to the appropriate libraries
target_link_libraries(bag_publisher ${catkin_LIBRARIES})
target_link_libraries(image_processor ${catkin_LIBRARIES})
target_link_libraries(kf_localizer ${catkin_LIBRARIES})

# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

