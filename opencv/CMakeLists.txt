add_library(apriltag_opencv apriltag_opencv.cpp)

add_executable(apriltag_opencv_demo apriltag_opencv_demo.cpp)
target_link_libraries(apriltag_opencv_demo apriltag
  apriltag_opencv ${CMAKE_THREAD_LIBS_INIT} ${OPENCV_LDFLAGS} m)

add_executable(box_test box_test.cpp)
target_link_libraries(box_test apriltag
  apriltag_opencv ${CMAKE_THREAD_LIBS_INIT} ${OPENCV_LDFLAGS} m)
