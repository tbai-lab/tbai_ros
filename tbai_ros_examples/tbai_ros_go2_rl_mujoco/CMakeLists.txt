cmake_minimum_required(VERSION 3.0.2)
project(tbai_ros_go2_rl_mujoco)

## C++17 required by tbai_sdk (zenoh)
add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
    tbai_ros_core
    tbai_ros_np3o
    tbai_ros_bob
    tbai_ros_wtw
    tbai_ros_static
    tbai_ros_reference
)

find_package(tbai REQUIRED)
find_package(spdlog REQUIRED)

# tbai_sdk (header-only, zenoh-based)
set(TBAI_SDK_DIR "$ENV{TBAI_SDK_DIR}" CACHE PATH "Path to tbai_sdk")
set(ZENOH_INSTALL "${TBAI_SDK_DIR}/deps/zenoh-install")
include_directories(${TBAI_SDK_DIR}/include ${ZENOH_INSTALL}/include)
add_definitions(-DZENOHCXX_ZENOHC)
link_directories(${ZENOH_INSTALL}/lib)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS tbai_ros_np3o tbai_ros_bob tbai_ros_wtw tbai_ros_static tbai_ros_reference
)


add_executable(deploy_go2_rl
    src/deploy_go2_rl.cpp
)
target_link_libraries(deploy_go2_rl
    PUBLIC
    ${catkin_LIBRARIES}
    tbai::core
    tbai::deploy_go2
    tbai::reference
    tbai::static
    zenohc
)
target_include_directories(deploy_go2_rl
    PRIVATE
    "${catkin_INCLUDE_DIRS}" include
)

