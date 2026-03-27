# Rewrite relative mesh paths in URDF files to use package:// URIs.
# Called with: -DURDF_DIR=<path> -DPACKAGE_NAME=<name>

file(GLOB ROBOT_DIRS "${URDF_DIR}/robots/*")
foreach(ROBOT_DIR ${ROBOT_DIRS})
  if(IS_DIRECTORY ${ROBOT_DIR})
    get_filename_component(ROBOT_NAME ${ROBOT_DIR} NAME)
    file(GLOB URDF_FILES "${ROBOT_DIR}/*.urdf")
    foreach(URDF_FILE ${URDF_FILES})
      file(READ ${URDF_FILE} CONTENT)
      string(REPLACE
        "./meshes/"
        "package://${PACKAGE_NAME}/urdf/robots/${ROBOT_NAME}/meshes/"
        CONTENT "${CONTENT}")
      file(WRITE ${URDF_FILE} "${CONTENT}")
    endforeach()
  endif()
endforeach()
