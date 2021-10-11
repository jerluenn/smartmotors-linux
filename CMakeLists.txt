cmake_minimum_required(VERSION 2.8.3)
project(smartmotors_linux)


find_package(catkin REQUIRED
	COMPONENTS
	roscpp
  	sensor_msgs
  	std_msgs
	geometry_msgs
	message_generation
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

add_message_files(

  FILES
  command.msg

)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
	INCLUDE_DIRS include 
	LIBRARIES smartmotors_linux
	CATKIN_DEPENDS 	roscpp sensor_msgs std_msgs geometry_msgs
	DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)


add_library(smartmotors_linux src/Class5LinuxApp.c)

# add_executable(testcpp src/smartmotor.cpp src/testcpp.cpp src/Class5LinuxApp.c)

add_executable(smartmotors src/smartmotor.cpp src/Class5LinuxApp.c src/wrapper.cpp)

target_link_libraries(smartmotors ${catkin_LIBRARIES})

install(TARGETS smartmotors_linux
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)



