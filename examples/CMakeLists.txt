add_subdirectory(core)
add_subdirectory(custom-node)

if(PCLGRAPH_TASKGRAPH_UI STREQUAL "imgui")
  add_subdirectory(ui)
endif()
