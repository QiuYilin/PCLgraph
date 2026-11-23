add_executable(pclgraph-ui-example main.cpp)
target_compile_definitions(pclgraph-ui-example PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
target_link_libraries(pclgraph-ui-example PRIVATE pclgraph::Nodes taskgraph::Ui)

# 模板留在源码树；运行时只打开 exe 旁副本，避免改写污染 Git。
add_custom_command(
  TARGET pclgraph-ui-example
  POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_directory
          "${CMAKE_CURRENT_SOURCE_DIR}/demo-project"
          "$<TARGET_FILE_DIR:pclgraph-ui-example>/demo-project"
  COMMENT "Copy pclgraph-ui-example demo-project next to executable"
)
