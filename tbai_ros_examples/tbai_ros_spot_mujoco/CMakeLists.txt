cmake_minimum_required(VERSION 3.14)
project(tbai_ros_spot_mujoco)

add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 17)

set(TBAI_SDK_DIR "$ENV{TBAI_SDK_DIR}" CACHE PATH "Path to tbai_sdk")
set(ZENOH_INSTALL "${TBAI_SDK_DIR}/deps/zenoh-install")
include_directories(${TBAI_SDK_DIR}/include ${ZENOH_INSTALL}/include)
add_definitions(-DZENOHCXX_ZENOHC)
link_directories(${ZENOH_INSTALL}/lib)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  sensor_msgs
  tbai_ros_core
  kdl_parser robot_state_publisher visualization_msgs
  tbai_ros_static
  tbai_ros_reference
  tbai_ros_mpc
  tbai_ros_ocs2
)

find_package(tbai REQUIRED)
find_package(spdlog REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp roslib std_msgs sensor_msgs tbai_ros_core tbai_ros_static tbai_ros_reference tbai_ros_mpc tbai_ros_ocs2
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(deploy_spot src/deploy_spot.cpp)
target_link_libraries(deploy_spot
  ${catkin_LIBRARIES}
  tbai::core
  tbai::deploy_spot
  tbai::reference
  tbai::static
  zenohc
  spdlog::spdlog
)

find_library(TBAI_ROS_QUADRUPED_ARM_MPC_LIB tbai_ros_quadruped_arm_mpc
  PATHS ${CATKIN_DEVEL_PREFIX}/lib ${CATKIN_DEVEL_PREFIX}/lib NO_DEFAULT_PATH)

add_executable(deploy_spot_arm src/deploy_spot_arm.cpp)
target_link_libraries(deploy_spot_arm
  ${TBAI_ROS_QUADRUPED_ARM_MPC_LIB}
  ${catkin_LIBRARIES}
  tbai::core
  tbai::deploy_spot
  tbai::reference
  tbai::static
  zenohc
  spdlog::spdlog
  yaml-cpp
)

install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(TARGETS deploy_spot deploy_spot_arm RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
