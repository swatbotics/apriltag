#include "apriltag_opencv.h"
#include "getopt.h"
#include "contour.h"
#include "box.h"
#include "g2d.h"
#include "apriltag_quad_contour.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <float.h>
#include <stdio.h>

#include "workerpool.h"
#include <pthread.h>




void test_integrate() {

  cv::Size img_sizes[] = {
    cv::Size(4, 3),
    //cv::Size(64, 48),
    //cv::Size(640, 480),
    //cv::Size(1280, 960),
    //cv::Size(2560, 1920),
    cv::Size(800, 600),
    cv::Size(0,0),
  };

  int lsizes[] = {
    0, 2, 7, -1
  };

  workerpool_t* wp = workerpool_create(4);

  for (int i=0; img_sizes[i].width; ++i) {

    cv::Size isz = img_sizes[i];
    
    for (int k=0; lsizes[k]>=0; ++k) {

      int l = lsizes[k];
      std::cout << "for size: " << isz << " with border " << l << "\n";
        
      for (int iter=0; iter<10; ++iter) {

        Mat8uc1 src(isz), src_b;
    
        cv::theRNG().fill(src, cv::RNG::UNIFORM, 0, 255);

        if (l) {
          cv::copyMakeBorder(src, src_b, l, l, l, l, cv::BORDER_REPLICATE);
        } else {
          src_b = src.clone();
        }
  
        Mat32sc1 ii_cv, ii_mz;

        image_u8_t src8 = cv2im8(src);

        timeprofile_t* tp = timeprofile_create();

        image_u32_t* ii = integrate_border_replicate(&src8, l);
                         
        timeprofile_stamp(tp, "mz integral");

        cv::integral(src_b, ii_cv);

        timeprofile_stamp(tp, "cv integral");

        timeprofile_display(tp);
        timeprofile_destroy(tp);

        ii_mz = im2cv(ii);

        Mat32sc1 diff;
        cv::absdiff(ii_cv, ii_mz, diff);

        double error = cv::sum(diff)[0];

        std::cout << "error = " << error << "\n\n";

        if (error) {
          std::cerr << "something went wrong, quitting\n";
          if (isz.width <= 16 && isz.height <= 16) {
            std::cerr << ii_cv << "\n";
            std::cerr << ii_mz << "\n";
          }
          exit(1);
        }

      }
    
    }

  }

  workerpool_destroy(wp);
  
}


void test_box() {

  cv::Size img_sizes[] = {
    cv::Size(4, 3),
    cv::Size(64, 48),
    cv::Size(640, 480),
    cv::Size(800, 600),
    cv::Size(1280, 960),
    cv::Size(0,0),
  };

  int box_sizes[] = {
    3, 5, 15, 0,
  };

  for (int i=0; img_sizes[i].width; ++i) {

    cv::Size isz(img_sizes[i]);

    for (int b=0; box_sizes[b]; ++b) {

      int bsz = box_sizes[b];

      Mat8uc1 orig(isz), mz_blur, cv_blur, diff;
      
      cv::theRNG().fill(orig, cv::RNG::UNIFORM, 0, 255);
      
      mz_blur = orig.clone();
    
      image_u8_t image = { mz_blur.cols, mz_blur.rows, mz_blur.cols, mz_blur.data };

      image_u8_t* imcpy = image_u8_copy(&image);

      std::cout << "for size " << isz << " with width " << bsz << "\n";

      timeprofile_t* tp = timeprofile_create();

      double sigma = double(bsz)/6.0;
      image_u8_gaussian_blur(imcpy, sigma, bsz);

      timeprofile_stamp(tp, "april blur");

      box_filter_border_replicate(&image, &image, bsz);

      timeprofile_stamp(tp, "mz blur");

      cv::blur(orig, cv_blur, cv::Size(bsz, bsz), 
               cv::Point(-1, -1), cv::BORDER_REPLICATE);

      timeprofile_stamp(tp, "cv blur");

      timeprofile_display(tp);
      timeprofile_destroy(tp);

      cv::absdiff(mz_blur, cv_blur, diff);

      double error = cv::sum(diff)[0];

      std::cout << "error = " << error << "\n\n";

      if (error) {
        std::cerr << "something went wrong, quitting\n";
        if (isz.width <= 16 && isz.height <= 16) {
          std::cerr << cv_blur << "\n";
          std::cerr << mz_blur << "\n";
        }
        exit(1);
      }

    }

  }

}


void test_outer() {

  int scl = 7;

  cv::Mat src = cv::imread("../thresh_tiny.png", CV_LOAD_IMAGE_GRAYSCALE);
  if (src.empty()) {
    fprintf(stderr, "nope!\n");
    exit(1);
  }
  
  image_u8_t s8 = cv2im8(src);

  std::cout << "src has size " << src.size() << "\n";

  zarray_t* contours = contour_detect(&s8);

  std::cout << "got " << zarray_size(contours) << " contours\n";

  cv::Mat big1;
  cv::resize(src, big1, src.size()*scl, 0, 0, cv::INTER_NEAREST);
  cv::Mat tmp;
  cv::cvtColor(big1, tmp, cv::COLOR_GRAY2RGB);
  big1 = tmp;

  big1 = big1/2 + 63;
  cv::Mat big2 = big1.clone();

  for (int i=0; i<zarray_size(contours); ++i) {
    const contour_info_t* ci;
    zarray_get_volatile(contours, i, &ci);
    
    if (ci->is_outer) {
      
      cv::Scalar color = random_color();
      polylines(big1, ci->points, color, true, scl);

      zarray_t* outer = contour_outer_boundary(ci, 0, zarray_size(ci->points));

      polylines(big2, outer, color, false, scl);

      zarray_destroy(outer);
      
    }

  }

  while (1) {
    cv::imshow("win", big1);
    cv::waitKey();
    cv::imshow("win", big2);
    cv::waitKey();
  }
    

}


int main(int argc, char *argv[]) {

  //test_outer();
  //test_box();
  test_integrate();

  return 0;

}
