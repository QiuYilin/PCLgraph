# Build-time download for the UI demo sample. Configure must not fail when the
# network is unavailable; only the UI example build depends on this file.

if(EXISTS "${OUTPUT}")
  file(SHA256 "${OUTPUT}" _existing_hash)
  if(_existing_hash STREQUAL "${SHA256}")
    return()
  endif()
  file(REMOVE "${OUTPUT}")
endif()

get_filename_component(_output_dir "${OUTPUT}" DIRECTORY)
file(MAKE_DIRECTORY "${_output_dir}")

set(_download "${OUTPUT}.download")
file(DOWNLOAD "${URL}" "${_download}"
     SHOW_PROGRESS STATUS _status TLS_VERIFY ON)
list(GET _status 0 _status_code)
if(NOT _status_code EQUAL 0)
  file(REMOVE "${_download}")
  message(WARNING
    "Failed to download chef.pcd (${_status}). "
    "pclgraph-ui-example will build without the demo sample. "
    "Set -DPCLGRAPH_DEMO_PCD=<local chef.pcd> or retry with HTTP(S)_PROXY.")
  return()
endif()

file(SHA256 "${_download}" _download_hash)
if(NOT _download_hash STREQUAL "${SHA256}")
  file(REMOVE "${_download}")
  message(WARNING
    "chef.pcd SHA256 mismatch: ${_download_hash}. "
    "pclgraph-ui-example will build without the demo sample. "
    "Set -DPCLGRAPH_DEMO_PCD=<local chef.pcd>.")
  return()
endif()

file(RENAME "${_download}" "${OUTPUT}")
