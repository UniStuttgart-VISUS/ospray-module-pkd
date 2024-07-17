ispc_include_directories(
  ${PROJECT_SOURCE_DIR}/ospray
  ${PROJECT_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${RKCOMMON_INCLUDE_DIRS}
  ${EMBREE_INCLUDE_DIRS}
)

add_library(ospray_module_pkd SHARED)

ispc_target_add_sources(ospray_module_pkd
  ${OSPRAY_RESOURCE}
  # the cpp file that contains all the plugin code - parsing
  # parameters in ospCommit(), creating and registering the object,
  # building accel structures, etc
  PKDGeometry.cpp

  # the ispc file that contains the plugins for all vector code - ie,
  # for ray-primitive intersection and 'postIntersect' (reporting info
  # on a previously computed ray-prim intersection)
  PKDGeometry.ispc

  # and finally, the module init code (not doing much, but must be there)
  moduleInit.cpp
)

target_link_libraries(ospray_module_pkd PRIVATE rkcommon::rkcommon ospray_module_cpu)

target_include_directories(ospray_module_pkd
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
  PRIVATE
    ${CMAKE_CURRENT_BINARY_DIR}
    # For ISPC module generated ispc headers
    ${PROJECT_BINARY_DIR}/ospray/
)
