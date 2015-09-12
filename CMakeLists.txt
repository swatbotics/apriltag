project(apriltag C CXX)

cmake_minimum_required ( VERSION 2.8 )
find_package ( Threads REQUIRED )

if(THREADS_HAVE_PTHREAD_ARG)
  add_compile_options("-pthread")
endif()

add_compile_options("-Wall")

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)

add_subdirectory(core)

