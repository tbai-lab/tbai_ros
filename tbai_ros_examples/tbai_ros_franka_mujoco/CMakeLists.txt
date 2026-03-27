cmake_minimum_required(VERSION 3.14)
project(tbai_ros_franka_mujoco)

add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 17)

# tbai_sdk (header-only, zenoh-based)
set(TBAI_SDK_DIR "/home/kb/projects/tbai_sdk")
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
  tbai_ros_static
  tbai_ros_mpc
  kdl_parser robot_state_publisher visualization_msgs
)

find_package(tbai REQUIRED)
find_package(spdlog REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp roslib std_msgs sensor_msgs tbai_ros_core tbai_ros_static tbai_ros_mpc
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# tbai_ros_arm_mpc is not exported by tbai_ros_mpc's catkin_package, link explicitly
find_library(TBAI_ROS_ARM_MPC_LIB tbai_ros_arm_mpc
  PATHS
    ${CATKIN_DEVEL_PREFIX}/lib
    ${CATKIN_DEVEL_PREFIX}/../lib
    /home/kb/projects/ros_test/devel/lib
  NO_DEFAULT_PATH
)

# Deployment executable (mirrors simple_arm_wbc but with FrankaRobotInterface)
add_executable(deploy_franka src/deploy_franka.cpp)
target_link_libraries(deploy_franka
  ${catkin_LIBRARIES}
  ${TBAI_ROS_ARM_MPC_LIB}
  tbai::core
  tbai::deploy_franka
  tbai::static
  spdlog::spdlog
  yaml-cpp
  zenohc
)

# Install
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(TARGETS deploy_franka RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
