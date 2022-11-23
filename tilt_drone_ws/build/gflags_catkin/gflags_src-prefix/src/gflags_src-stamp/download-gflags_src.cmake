if(EXISTS "/home/radu/tiltUp3_ws/build/gflags_catkin/gflags_src-prefix/src/v2.2.1.zip")
  file("MD5" "/home/radu/tiltUp3_ws/build/gflags_catkin/gflags_src-prefix/src/v2.2.1.zip" hash_value)
  if("x${hash_value}" STREQUAL "x2d988ef0b50939fb50ada965dafce96b")
    return()
  endif()
endif()
message(STATUS "downloading...
     src='https://github.com/gflags/gflags/archive/v2.2.1.zip'
     dst='/home/radu/tiltUp3_ws/build/gflags_catkin/gflags_src-prefix/src/v2.2.1.zip'
     timeout='none'")




file(DOWNLOAD
  "https://github.com/gflags/gflags/archive/v2.2.1.zip"
  "/home/radu/tiltUp3_ws/build/gflags_catkin/gflags_src-prefix/src/v2.2.1.zip"
  SHOW_PROGRESS
  # no TIMEOUT
  STATUS status
  LOG log)

list(GET status 0 status_code)
list(GET status 1 status_string)

if(NOT status_code EQUAL 0)
  message(FATAL_ERROR "error: downloading 'https://github.com/gflags/gflags/archive/v2.2.1.zip' failed
  status_code: ${status_code}
  status_string: ${status_string}
  log: ${log}
")
endif()

message(STATUS "downloading... done")
