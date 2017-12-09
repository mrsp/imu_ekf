cmake_minimum_required(VERSION 2.8.3)
project(imu_ekf)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  dynamic_reconfigure
)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)


## Generate dynamic reconfigure parameters in the 'cfg' folder
 generate_dynamic_reconfigure_options(
   cfg/ParamControl.cfg
 )

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES humanoid_state_estimation
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs tf 
  DEPENDS Eigen3
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)


add_executable(imu_ekf src/imu_estimator_driver.cpp src/imu_estimator.cpp  src/IMUEKF.cpp)
target_link_libraries(imu_ekf ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})

add_dependencies(imu_ekf ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

