cmake_minimum_required(VERSION 3.22)
project(tbai_ros_mujoco)

find_package(catkin REQUIRED)
find_package(Git REQUIRED)

catkin_package()

include(ExternalProject)

# ── Find or clone tbai_mujoco ─────────────────────────────────────────────
set(TBAI_MUJOCO_DIR "$ENV{TBAI_MUJOCO_ROOT}" CACHE PATH "Path to tbai_mujoco")
if(NOT EXISTS "${TBAI_MUJOCO_DIR}/CMakeLists.txt")
  set(TBAI_MUJOCO_DIR ${CMAKE_CURRENT_BINARY_DIR}/tbai_mujoco)
  if(NOT EXISTS ${TBAI_MUJOCO_DIR})
    message(STATUS "Cloning tbai_mujoco...")
    execute_process(
      COMMAND ${GIT_EXECUTABLE} clone --depth 1 git@github.com:tbai-lab/tbai_mujoco.git ${TBAI_MUJOCO_DIR}
      RESULT_VARIABLE GIT_RESULT
    )
    if(NOT GIT_RESULT EQUAL 0)
      message(FATAL_ERROR "Failed to clone tbai_mujoco")
    endif()
  endif()
else()
  message(STATUS "Using tbai_mujoco from TBAI_MUJOCO_ROOT: ${TBAI_MUJOCO_DIR}")
endif()

# ── Clone tbai_sdk if needed ──────────────────────────────────────────────
set(TBAI_SDK_DIR "$ENV{TBAI_SDK_DIR}" CACHE PATH "Path to tbai_sdk")
if(NOT EXISTS "${TBAI_SDK_DIR}/include/tbai_sdk/session.hpp")
  set(TBAI_SDK_DIR ${CMAKE_CURRENT_BINARY_DIR}/tbai_sdk)
  if(NOT EXISTS ${TBAI_SDK_DIR})
    message(STATUS "Cloning tbai_sdk...")
    execute_process(
      COMMAND ${GIT_EXECUTABLE} clone --depth 1 git@github.com:tbai-lab/tbai_sdk.git ${TBAI_SDK_DIR}
      RESULT_VARIABLE GIT_RESULT
    )
    if(NOT GIT_RESULT EQUAL 0)
      message(FATAL_ERROR "Failed to clone tbai_sdk")
    endif()
  endif()
endif()

# ── Build tbai_mujoco using its own CMake ─────────────────────────────────
ExternalProject_Add(tbai_mujoco_build
  SOURCE_DIR ${TBAI_MUJOCO_DIR}
  CMAKE_ARGS
    -DCMAKE_BUILD_TYPE=Release
    -DTBAI_SDK_DIR=${TBAI_SDK_DIR}
    -DCMAKE_INSTALL_PREFIX=${CATKIN_DEVEL_PREFIX}
  BUILD_ALWAYS OFF
  INSTALL_COMMAND ""
)

# ── Copy binary + plugins to devel space ──────────────────────────────────
ExternalProject_Get_Property(tbai_mujoco_build BINARY_DIR)

add_custom_target(install_tbai_mujoco ALL
  DEPENDS tbai_mujoco_build
  COMMAND ${CMAKE_COMMAND} -E make_directory ${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME}
  COMMAND ${CMAKE_COMMAND} -E copy
    ${BINARY_DIR}/simulate/tbai_mujoco
    ${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME}/tbai_mujoco
  COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${BINARY_DIR}/simulate/mujoco_plugin
    ${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME}/mujoco_plugin
  COMMENT "Installing tbai_mujoco to devel space"
)
