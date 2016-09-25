set(sources
  apriltag.c apriltag_quad_thresh.c tag16h5.c tag25h7.c tag25h9.c 
  tag36h10.c tag36h11.c tag36artoolkit.c g2d.c apriltag_family.c
  common/zarray.c 
  common/zhash.c common/zmaxheap.c common/unionfind.c common/matd.c 
  common/image_u8.c common/pnm.c common/image_f32.c common/image_u32.c 
  common/workerpool.c common/time_util.c common/svd22.c 
  common/homography.c common/string_util.c common/getopt.c
  contrib/box.c contrib/contour.c contrib/lm.c contrib/pdfutil.c
  contrib/apriltag_quad_contour.c contrib/apriltag_vis.c)

include_directories(common contrib .)

add_library(apriltag SHARED ${sources})

add_executable(apriltag_demo apriltag_demo.c)
target_link_libraries(apriltag_demo apriltag ${CMAKE_THREAD_LIBS_INIT} m)

add_executable(contour_test contrib/contour_test.c)
target_link_libraries(contour_test apriltag ${CMAKE_THREAD_LIBS_INIT} m)

add_executable(lm_test contrib/lm_test.cpp)
target_link_libraries(lm_test apriltag ${CMAKE_THREAD_LIBS_INIT} m)

add_executable(pdf_test contrib/pdf_test.cpp)
target_link_libraries(pdf_test apriltag ${CMAKE_THREAD_LIBS_INIT} m)

install(TARGETS apriltag_demo DESTINATION bin)
install(TARGETS apriltag DESTINATION lib)
