cmake_minimum_required(VERSION 3.18)

project(pclgraph VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
  add_compile_options(/utf-8 /bigobj)
endif()

# 调试期直接吃本地 taskgraph 源码；发布期走 registry 的 find_package。
set(PCLGRAPH_TASKGRAPH_DIR "" CACHE PATH "Local taskgraph source dir (dev only)")
set(PCLGRAPH_TASKGRAPH_UI "none" CACHE STRING "taskgraph UI frontend: imgui, qt, none")
if(PCLGRAPH_TASKGRAPH_DIR)
  set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
  set(TASKGRAPH_UI "${PCLGRAPH_TASKGRAPH_UI}" CACHE STRING "" FORCE)
  add_subdirectory(${PCLGRAPH_TASKGRAPH_DIR} ${CMAKE_BINARY_DIR}/_taskgraph)
else()
  if(PCLGRAPH_TASKGRAPH_UI STREQUAL "none")
    find_package(taskgraph CONFIG REQUIRED COMPONENTS Core)
  else()
    find_package(taskgraph CONFIG REQUIRED COMPONENTS Core Ui)
  endif()
endif()

enable_testing()

find_package(PCL CONFIG REQUIRED COMPONENTS common filters io features search
             kdtree registration keypoints)
find_package(GTest CONFIG REQUIRED)

# 依赖分层：Nodes -> Core -> 本地 taskgraph。
add_library(pclgraph_core STATIC
  src/algorithm_registry.cpp
  src/point_type.cpp)
add_library(pclgraph::Core ALIAS pclgraph_core)
target_include_directories(pclgraph_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(pclgraph_core PUBLIC taskgraph::Core pcl_common)

add_library(pclgraph_nodes STATIC
  src/nodes/pcl_node_base.cpp
  src/nodes/point_cloud_bilateral_nodes.cpp
  src/nodes/point_cloud_calc_model_reso_node.cpp
  src/nodes/point_cloud_convert_node.cpp
  src/nodes/point_cloud_feature_nodes.cpp
  src/nodes/point_cloud_filter_nodes.cpp
  src/nodes/point_cloud_filter_nodes_extra.cpp
  src/nodes/point_cloud_fpfh_node.cpp
  src/nodes/point_cloud_io_extra_nodes.cpp
  src/nodes/point_cloud_io_nodes.cpp
  src/nodes/point_cloud_keypoint_nodes.cpp
  src/nodes/point_cloud_normals_filter_nodes.cpp
  src/nodes/point_cloud_registration_nodes.cpp
  src/nodes/point_cloud_transform_node.cpp
  src/nodes/register_nodes.cpp)
add_library(pclgraph::Nodes ALIAS pclgraph_nodes)
target_link_libraries(pclgraph_nodes PUBLIC
  pclgraph::Core
  pcl_filters
  pcl_io
  pcl_features
  pcl_keypoints
  pcl_search
  pcl_kdtree
  pcl_registration)

# 最小消费者：走 Nodes -> Core -> taskgraph 的完整链路。
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
add_executable(pclgraph_smoke src/smoke.cpp)
target_link_libraries(pclgraph_smoke PRIVATE pclgraph::Nodes)

add_executable(pclgraph_tests
  tests/core_test.cpp
  tests/nodes_test.cpp)
target_link_libraries(pclgraph_tests PRIVATE pclgraph::Nodes GTest::gtest_main)
add_test(NAME pclgraph_core_tests
  COMMAND ${CMAKE_COMMAND} -E env
    "PATH=$<IF:$<CONFIG:Debug>,C:/vcpkg/installed/x64-windows/debug/bin,C:/vcpkg/installed/x64-windows/bin>;${CMAKE_BINARY_DIR}/bin/$<CONFIG>;$ENV{PATH}"
    $<TARGET_FILE:pclgraph_tests>)

option(PCLGRAPH_BUILD_EXAMPLES "Build PCLgraph examples" ON)
if(PCLGRAPH_BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()

# Doxygen 默认关闭；只有显式开启且找到 Doxygen 时才创建 doc 目标。
option(PCLGRAPH_BUILD_DOCS "Build API documentation with Doxygen" OFF)
if(PCLGRAPH_BUILD_DOCS)
  find_package(Doxygen QUIET)
  if(NOT Doxygen_FOUND)
    message(FATAL_ERROR "PCLGRAPH_BUILD_DOCS=ON requires Doxygen")
  endif()
  set(DOXYGEN_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/docs")
  set(DOXYGEN_INPUT
      "\"${PROJECT_SOURCE_DIR}/include/pclgraph\" \"${PROJECT_SOURCE_DIR}/examples/custom-node\"")
  configure_file("${PROJECT_SOURCE_DIR}/docs/Doxyfile.in"
                 "${CMAKE_CURRENT_BINARY_DIR}/Doxyfile" @ONLY)
  add_custom_target(doc
    COMMAND ${DOXYGEN_EXECUTABLE} "${CMAKE_CURRENT_BINARY_DIR}/Doxyfile"
    WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
    COMMENT "Generating API documentation with Doxygen"
    VERBATIM)
endif()
