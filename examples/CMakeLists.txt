add_subdirectory(core)

if(PCLGRAPH_TASKGRAPH_UI STREQUAL "imgui")
  add_subdirectory(ui)
endif()
