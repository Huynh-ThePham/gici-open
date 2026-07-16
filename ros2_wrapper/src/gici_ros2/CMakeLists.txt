cmake_minimum_required(VERSION 3.8)
project(gici_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ament / ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(gici_ros2_msgs REQUIRED)

# Third-party dependencies (same as the GICI core)
find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(yaml-cpp REQUIRED)

# Build the GICI core library (target: gici). This mirrors the ROS1 wrapper,
# which did add_subdirectory(../../.. gici.out). Core sources stay byte-identical
# to upstream f2b8579; only this out-of-tree wrapper is added.
#
# GICI_ROOT defaults to the standard gici_research_standard monorepo layout
# (ros2_wrapper/src/gici_ros2/../../..), but can be overridden -- pass
# -DGICI_ROOT=/path/to/core, or set the GICI_CORE_ROOT environment variable --
# for a colcon workspace that vendors/symlinks this package independently of
# that layout. Validated below so a mismatch fails with an actionable message
# instead of an obscure "file not found" deep inside add_subdirectory().
if(NOT DEFINED GICI_ROOT)
  if(DEFINED ENV{GICI_CORE_ROOT})
    set(GICI_ROOT "$ENV{GICI_CORE_ROOT}")
  else()
    get_filename_component(GICI_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
  endif()
endif()
if(NOT EXISTS "${GICI_ROOT}/CMakeLists.txt" OR NOT EXISTS "${GICI_ROOT}/include/gici")
  message(FATAL_ERROR
    "GICI_ROOT (${GICI_ROOT}) does not look like the gici core source tree "
    "(expected CMakeLists.txt and include/gici/ there). This package expects "
    "to be built from within the gici_research_standard monorepo "
    "(ros2_wrapper/src/gici_ros2/); if this package has been relocated, pass "
    "-DGICI_ROOT=/path/to/core or set the GICI_CORE_ROOT environment "
    "variable before configuring.")
endif()
add_subdirectory(${GICI_ROOT} ${CMAKE_CURRENT_BINARY_DIR}/gici_core)

# Pin Eigen alignment to 16 bytes on EVERY target so the core (built with -msse)
# and this executable (which may inherit CXXFLAGS=-march=native -> AVX/32-byte) share
# one Eigen ABI. Without this, Eigen members inside DataCluster/kindr transforms are
# laid out differently across the DSO boundary and the node segfaults (kindr NCamera
# on startup, DataCluster destructor at runtime).
foreach(_t gici vikit_common svo fast)
  if(TARGET ${_t})
    target_compile_definitions(${_t} PUBLIC
      EIGEN_MAX_ALIGN_BYTES=16 EIGEN_MAX_STATIC_ALIGN_BYTES=16)
  endif()
endforeach()

# RTKLIB memory-layout definitions MUST match the core to keep struct sizes identical
add_definitions(-DENAGLO -DENACMP -DENAGAL -DENAQZS -DENAIRN -DNFREQ=3 -DNEXOBS=3 -DDLL)
add_definitions(-DEIGEN_MAX_ALIGN_BYTES=16 -DEIGEN_MAX_STATIC_ALIGN_BYTES=16)

add_executable(gici_ros2_main
  src/gici_ros2_main.cpp
  src/ros_interface/ros_node_handle.cpp
  src/ros_interface/ros_stream.cpp
  src/ros_interface/ros_publisher.cpp)

target_compile_options(gici_ros2_main PRIVATE -fpermissive -Wno-unused)

target_include_directories(gici_ros2_main PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS})

# gici (PUBLIC) transitively provides rtklib / svo / vikit / glog includes+libs
target_link_libraries(gici_ros2_main
  gici
  ${OpenCV_LIBS}
  yaml-cpp)

ament_target_dependencies(gici_ros2_main
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  cv_bridge
  gici_ros2_msgs)

# The GICI core libraries are built out-of-tree (add_subdirectory) and are not part of
# the ament install space. Install them next to the executable and resolve them via an
# $ORIGIN rpath so the node runs without setting LD_LIBRARY_PATH.
set(GICI_CORE_TARGETS gici rtklib svo vikit_common fast)
foreach(_t ${GICI_CORE_TARGETS})
  set_target_properties(${_t} PROPERTIES INSTALL_RPATH "$ORIGIN")
endforeach()
set_target_properties(gici_ros2_main PROPERTIES INSTALL_RPATH "$ORIGIN")

install(TARGETS gici_ros2_main ${GICI_CORE_TARGETS}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
