cmake_minimum_required(VERSION 3.0.2)
project(tbai_ros_descriptions)

find_package(catkin REQUIRED)
find_package(Git REQUIRED)

catkin_package()

# Clone repos at build time into the build directory
set(DESCRIPTIONS_DIR ${CMAKE_CURRENT_BINARY_DIR}/tbai_descriptions)
set(MUJOCO_DESCRIPTIONS_DIR ${CMAKE_CURRENT_BINARY_DIR}/tbai_mujoco_descriptions)

if(NOT EXISTS ${DESCRIPTIONS_DIR})
  message(STATUS "Cloning tbai_descriptions...")
  execute_process(
    COMMAND ${GIT_EXECUTABLE} clone --depth 1 git@github.com:tbai-lab/tbai_descriptions.git ${DESCRIPTIONS_DIR}
    RESULT_VARIABLE GIT_RESULT
  )
  if(NOT GIT_RESULT EQUAL 0)
    message(FATAL_ERROR "Failed to clone tbai_descriptions")
  endif()
endif()

if(NOT EXISTS ${MUJOCO_DESCRIPTIONS_DIR})
  message(STATUS "Cloning tbai_mujoco_descriptions...")
  execute_process(
    COMMAND ${GIT_EXECUTABLE} clone --depth 1 git@github.com:tbai-lab/tbai_mujoco_descriptions.git ${MUJOCO_DESCRIPTIONS_DIR}
    RESULT_VARIABLE GIT_RESULT
  )
  if(NOT GIT_RESULT EQUAL 0)
    message(FATAL_ERROR "Failed to clone tbai_mujoco_descriptions")
  endif()
endif()

# Rewrite relative mesh paths in URDFs to package:// URIs
execute_process(
  COMMAND ${CMAKE_COMMAND}
    -DURDF_DIR=${DESCRIPTIONS_DIR}
    -DPACKAGE_NAME=${PROJECT_NAME}
    -P ${CMAKE_CURRENT_SOURCE_DIR}/fix_urdf_paths.cmake
)

# Install URDF descriptions (robots/ and assets/)
install(DIRECTORY ${DESCRIPTIONS_DIR}/robots/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf/robots
)
install(DIRECTORY ${DESCRIPTIONS_DIR}/assets/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf/assets
)

# Install MuJoCo descriptions (robots/, worlds/, and assets/)
install(DIRECTORY ${MUJOCO_DESCRIPTIONS_DIR}/robots/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/mujoco/robots
)
install(DIRECTORY ${MUJOCO_DESCRIPTIONS_DIR}/worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/mujoco/worlds
)
install(DIRECTORY ${MUJOCO_DESCRIPTIONS_DIR}/assets/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/mujoco/assets
)

# Copy into source tree so package:// URIs resolve via rospack find
file(COPY ${DESCRIPTIONS_DIR}/robots/ DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/urdf/robots)
file(COPY ${DESCRIPTIONS_DIR}/assets/ DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/urdf/assets)
file(COPY ${MUJOCO_DESCRIPTIONS_DIR}/robots/ DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/mujoco/robots)
file(COPY ${MUJOCO_DESCRIPTIONS_DIR}/worlds/ DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/mujoco/worlds)
file(COPY ${MUJOCO_DESCRIPTIONS_DIR}/assets/ DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/mujoco/assets)
