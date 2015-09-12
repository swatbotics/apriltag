set(sources
  apriltag.c apriltag_quad_thresh.c tag16h5.c tag25h7.c tag25h9.c 
  tag36h10.c tag36h11.c tag36artoolkit.c g2d.c common/zarray.c 
  common/zhash.c common/zmaxheap.c common/unionfind.c common/matd.c 
  common/image_u8.c common/pnm.c common/image_f32.c common/image_u32.c 
  common/workerpool.c common/time_util.c common/svd22.c 
  common/homography.c common/string_util.c common/getopt.c)

include_directories(common)

add_library(apriltag ${sources})

add_executable(apriltag_demo apriltag_demo.c)
target_link_libraries(apriltag_demo apriltag ${CMAKE_THREAD_LIBS_INIT} m)

