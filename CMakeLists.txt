cmake_minimum_required(VERSION 2.8.3)
project(smartmotors_linux)


set(CMAKE_BUILD_TYPE Debug)
add_compile_options(-std=c++14)
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
  arraycommand.msg
  command.msg

)

add_service_files(
  FILES
  emergencyStop.srv
  setAllMotorTensions.srv
  setSpecificMotorTension.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
	INCLUDE_DIRS include 
	LIBRARIES smartmotors_linux
	CATKIN_DEPENDS 	roscpp sensor_msgs std_msgs geometry_msgs message_runtime
	DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(smartmotors_linux src/Class5LinuxApp.c)
add_library(MedianFilter src/medianfilter.cpp)
add_library(KalmanFilter src/kalmanfilter.cpp)

add_executable(smartmotors src/smartmotor.cpp src/wrapper.cpp)
add_executable(pid_tension src/pid_tension.cpp)
add_executable(pid_tension_test src/pid_tension_test.cpp)
add_executable(testMedianFilter src/testMedianFilter.cpp)
add_executable(testKalmanFilter src/testKalmanFilter.cpp)
add_executable(tension_rate_pid src/tension_rate_pid.cpp)

target_link_libraries(testKalmanFilter ${catkin_LIBRARIES} KalmanFilter)
target_link_libraries(testMedianFilter ${catkin_LIBRARIES} MedianFilter)
target_link_libraries(smartmotors ${catkin_LIBRARIES} smartmotors_linux)
target_link_libraries(pid_tension ${catkin_LIBRARIES} smartmotors_linux)
target_link_libraries(pid_tension_test ${catkin_LIBRARIES} smartmotors_linux KalmanFilter)
target_link_libraries(tension_rate_pid ${catkin_LIBRARIES} smartmotors_linux MedianFilter KalmanFilter)

install(TARGETS smartmotors_linux
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)



