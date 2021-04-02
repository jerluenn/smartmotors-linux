cmake_minimum_required(VERSION 2.8.3)
project(smartmotors-linux)


find_package(catkin REQUIRED
	COMPONENTS
	roscpp
  	sensor_msgs
  	std_msgs
	geometry_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

## Find Eigen
find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
endif()

catkin_package(
	INCLUDE_DIRS include 
	LIBRARIES smartmotors-linux
	CATKIN_DEPENDS 	roscpp sensor_msgs std_msgs geometry_msgs
	DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(smartmotors-linux src/Class5LinuxApp.c)

add_executable(testcpp src/testcpp.cpp src/Class5LinuxApp.c)

install(TARGETS smartmotors-linux
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

