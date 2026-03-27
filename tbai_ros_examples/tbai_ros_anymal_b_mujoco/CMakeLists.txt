cmake_minimum_required(VERSION 3.14)
project(tbai_ros_anymal_b_mujoco)

add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 17)

set(TBAI_SDK_DIR "$ENV{TBAI_SDK_DIR}" CACHE PATH "Path to tbai_sdk")
set(ZENOH_INSTALL "${TBAI_SDK_DIR}/deps/zenoh-install")
include_directories(${TBAI_SDK_DIR}/include ${ZENOH_INSTALL}/include)
add_definitions(-DZENOHCXX_ZENOHC)
link_directories(${ZENOH_INSTALL}/lib)

# Find Torch before catkin to avoid CGAL/MKL cmake conflict
find_package(Torch REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp roslib std_msgs sensor_msgs
  tbai_ros_core tbai_ros_static tbai_ros_reference
  tbai_ros_mpc tbai_ros_ocs2
  kdl_parser robot_state_publisher visualization_msgs
)

find_package(tbai REQUIRED)
find_package(spdlog REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp roslib std_msgs sensor_msgs
    tbai_ros_core tbai_ros_static tbai_ros_reference tbai_ros_mpc tbai_ros_ocs2
)

include_directories(${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

# MPC deploy
add_executable(deploy_anymal_b_mpc src/deploy_anymal_b_mpc.cpp)
target_link_libraries(deploy_anymal_b_mpc
  ${catkin_LIBRARIES} tbai::core tbai::deploy_anymal_b tbai::reference tbai::static
  zenohc spdlog::spdlog)

install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(TARGETS deploy_anymal_b_mpc
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
