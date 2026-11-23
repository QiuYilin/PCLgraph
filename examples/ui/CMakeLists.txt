find_package(imgui-vtk CONFIG REQUIRED)

# 示例点云不提交到仓库：默认下载 PCL 1.15.1 的 chef.pcd，或由调用方提供本地文件。
set(PCLGRAPH_DEMO_PCD "" CACHE FILEPATH
    "Local chef.pcd for the UI demo; empty downloads the pinned PCL sample")
set(_pclgraph_demo_pcd_url
    "https://raw.githubusercontent.com/PointCloudLibrary/pcl/pcl-1.15.1/doc/tutorials/content/sources/alignment_prerejective/chef.pcd")
set(_pclgraph_demo_pcd_sha256
    "a4924fc43c78c07d3fba6a0b7226ccf5c2134ea0a2e3e50d5e65b40c2f09fdfc")

set(_pclgraph_demo_pcd "${PCLGRAPH_DEMO_PCD}")
if(NOT "${_pclgraph_demo_pcd}" STREQUAL "" AND
   NOT IS_ABSOLUTE "${_pclgraph_demo_pcd}")
  get_filename_component(_pclgraph_demo_pcd "${_pclgraph_demo_pcd}" ABSOLUTE
                         BASE_DIR "${CMAKE_SOURCE_DIR}")
endif()
if(_pclgraph_demo_pcd STREQUAL "")
  set(_pclgraph_demo_pcd "${CMAKE_BINARY_DIR}/demo-data/chef.pcd")
  if(NOT EXISTS "${_pclgraph_demo_pcd}")
    file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/demo-data")
    set(_pclgraph_demo_download "${_pclgraph_demo_pcd}.download")
    file(DOWNLOAD "${_pclgraph_demo_pcd_url}" "${_pclgraph_demo_download}"
         SHOW_PROGRESS STATUS _pclgraph_demo_status)
    list(GET _pclgraph_demo_status 0 _pclgraph_demo_status_code)
    if(NOT _pclgraph_demo_status_code EQUAL 0)
      file(REMOVE "${_pclgraph_demo_download}")
      message(FATAL_ERROR
        "Failed to download chef.pcd (${_pclgraph_demo_status}). "
        "Set -DPCLGRAPH_DEMO_PCD=<local chef.pcd> or retry with HTTP(S)_PROXY.")
    endif()
    file(SHA256 "${_pclgraph_demo_download}" _pclgraph_demo_hash)
    if(NOT _pclgraph_demo_hash STREQUAL _pclgraph_demo_pcd_sha256)
      file(REMOVE "${_pclgraph_demo_download}")
      message(FATAL_ERROR
        "chef.pcd SHA256 mismatch: ${_pclgraph_demo_hash}. "
        "Set -DPCLGRAPH_DEMO_PCD=<local chef.pcd>.")
    endif()
    file(RENAME "${_pclgraph_demo_download}" "${_pclgraph_demo_pcd}")
  endif()
endif()
if(NOT EXISTS "${_pclgraph_demo_pcd}")
  message(FATAL_ERROR "PCLGRAPH_DEMO_PCD does not exist: ${_pclgraph_demo_pcd}")
endif()
message(STATUS "PCLgraph UI demo sample: ${_pclgraph_demo_pcd}")

add_executable(pclgraph-ui-example main.cpp point_cloud_view.cpp)
target_compile_definitions(pclgraph-ui-example PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
target_link_libraries(pclgraph-ui-example PRIVATE
  pclgraph::Nodes
  taskgraph::Ui
  imgui-vtk::imgui_vtk)
imgui_vtk_autoinit(pclgraph-ui-example)

# 模板留在源码树；运行时只打开 exe 旁副本，避免改写污染 Git。
add_custom_command(
  TARGET pclgraph-ui-example
  POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_directory
          "${CMAKE_CURRENT_SOURCE_DIR}/demo-project"
          "$<TARGET_FILE_DIR:pclgraph-ui-example>/demo-project"
  COMMAND ${CMAKE_COMMAND} -E rm -f
          "$<TARGET_FILE_DIR:pclgraph-ui-example>/demo-project/points.pcd"
          "$<TARGET_FILE_DIR:pclgraph-ui-example>/demo-project/out.pcd"
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
          "${_pclgraph_demo_pcd}"
          "$<TARGET_FILE_DIR:pclgraph-ui-example>/demo-project/chef.pcd"
  COMMENT "Copy pclgraph-ui-example demo-project and sample PCD next to executable"
)
