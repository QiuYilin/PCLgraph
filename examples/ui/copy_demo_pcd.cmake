if(EXISTS "${SOURCE}")
  execute_process(
    COMMAND ${CMAKE_COMMAND} -E copy_if_different "${SOURCE}" "${DEST}"
    RESULT_VARIABLE _copy_result)
  if(NOT _copy_result EQUAL 0)
    message(FATAL_ERROR "Failed to copy chef.pcd to ${DEST}")
  endif()
else()
  message(WARNING
    "chef.pcd is not available; pclgraph-ui-example will run without the "
    "demo sample. Set -DPCLGRAPH_DEMO_PCD=<local chef.pcd> or retry with "
    "HTTP(S)_PROXY.")
endif()
