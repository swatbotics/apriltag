#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "contour.h"
#include "apriltag_opencv.h"
#include "time_util.h"
#include <iostream>

int main(int argc, char** argv) {


  cv::Size size(800, 600);
  int scl = 20;
  
  cv::Size ssize(size.width/scl, size.height/scl);
  int t0 = 220;
  int bsz = 55;
  int t1 = 0;

  cv::RNG rng(1234567);

  uint64_t total0 = 0;
  uint64_t total1 = 0;

  const int maxiter = 100;

  for (int iter=0; iter<maxiter; ++iter) {

    cv::Mat_<uint8_t> small(ssize);
    rng.fill(small, cv::RNG::UNIFORM, 0, 255);

    small = (small > t0) * 255;

    cv::Mat_<uint8_t> large, lblur;

    cv::resize(small, large, size, 0.0, 0.0, cv::INTER_CUBIC);

    cv::adaptiveThreshold(large, lblur, 255, cv::ADAPTIVE_THRESH_MEAN_C,
                          cv::THRESH_BINARY, bsz, t1);

    lblur.col(0) = 0;
    lblur.col(size.width-1) = 0;

    lblur.row(0) = 0;
    lblur.row(size.height-1) = 0;

    //cv::imshow("win", lblur); cv::waitKey();

    image_u8_t im8 = cv2im8(lblur);

    uint64_t start;

    start = utime_now();
    zarray_t* c0 = contour_detect(&im8);
    total0 += (utime_now() - start);
    

    start = utime_now();
    zarray_t* c1 = contour_line_sweep(&im8);
    total1 += (utime_now() - start);
               

    int s0 = zarray_size(c0);
    int s1 = zarray_size(c1);
    std::cout << "s0 = " << s0 << ", s1 = " << s1 << "\n";

    contour_destroy(c0);
    contour_line_sweep_destroy(c1);
    
  }

  std::cout << "total0 = " << total0 << " (" << (total0*1e-3)/maxiter << "ms per)\n";
  std::cout << "total1 = " << total1 << " (" << (total1*1e-3)/maxiter << "ms per)\n";

  return 0;

}
