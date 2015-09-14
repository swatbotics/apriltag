project(apriltag C CXX)

cmake_minimum_required ( VERSION 2.8 )
find_package ( Threads REQUIRED )

if(THREADS_HAVE_PTHREAD_ARG)
  add_compile_options("-pthread")
endif()

add_compile_options("-Wall")
add_compile_options("-g")


set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)

add_subdirectory(core)
include_directories(core core/common core/contrib)

include(FindPkgConfig)
pkg_search_module(OPENCV opencv>=2.3 opencv-2.3.1 opencv-3.0.0)

if(OPENCV_FOUND)
  include_directories(${OPENCV_INCLUDE_DIRS})
  add_subdirectory(opencv)
endif(OPENCV_FOUND)
