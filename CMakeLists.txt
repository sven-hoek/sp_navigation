cmake_minimum_required(VERSION 2.8.3)
project(sp_navigation)
SET(CMAKE_MODULE_PATH /usr/local/lib/cmake/ )
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  tf
  pcl_ros
  image_transport
  image_geometry
  cv_bridge
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(cvsba REQUIRED)

catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_executable(sp_navigation src/sp_navigation.cpp)
target_link_libraries(sp_navigation ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${cvsba_LIBS} ${PCL_LIBRARIES})
add_dependencies(sp_navigation sp_navigation_generate_messages_cpp)
