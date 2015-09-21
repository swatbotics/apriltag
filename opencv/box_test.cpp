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

const cv::Size img_sizes[] = {
  //cv::Size(4, 3),
  //cv::Size(64, 48),
  //cv::Size(640, 480),
  cv::Size(800, 600),
  //cv::Size(1280, 960),
  //cv::Size(2560, 1920),
  cv::Size(0,0),
};

const int box_sizes[] = {
  //1, 5, 15, 0,
  15, 0
};

const int thresholds[] = {
  0, 3, 5, 256
};

const int max_iter = 100;

void test_integrate() {

  workerpool_t* wp = workerpool_create(4);

  for (int i=0; img_sizes[i].width; ++i) {

    cv::Size isz = img_sizes[i];
    
    for (int b=0; box_sizes[b]; ++b) {

      int l = box_sizes[b]/2;

      std::cout << "integrating size " << isz << " with border " << l << "\n";
        
      Mat8uc1 src(isz), src_b;
    
      cv::theRNG().fill(src, cv::RNG::UNIFORM, 0, 255);
      
      if (l) {
        cv::copyMakeBorder(src, src_b, l, l, l, l, cv::BORDER_REPLICATE);
      } else {
        src_b = src.clone();
      }
      
      Mat32sc1 ii_cv, ii_mz, ii_mz_mt;
      
      image_u8_t src8 = cv2im8(src);
      
      image_u32_t* ii = NULL, *ii_mt = NULL;

      int64_t elapsed_mz=0, elapsed_mz_mt=0, elapsed_cv=0;

      
      for (int iter=0; iter<max_iter; ++iter) {
        if (ii) { image_u32_destroy(ii); }
        int64_t start = utime_now();
        ii = integrate_border_replicate(&src8, l);
        elapsed_mz += utime_now() - start;
      }

      for (int iter=0; iter<max_iter; ++iter) {
        if (ii_mt) { image_u32_destroy(ii_mt); }
        int64_t start = utime_now();
        ii_mt = integrate_border_replicate_mt(&src8, l, wp);
        elapsed_mz_mt += utime_now() - start;
      }
        
      for (int iter=0; iter<max_iter; ++iter) {
        int64_t start = utime_now();
        cv::integral(src_b, ii_cv);
        elapsed_cv += utime_now() - start;
      }

      
      
      ii_mz = im2cv(ii);
      ii_mz_mt = im2cv(ii_mt);
      
      
      Mat32sc1 diff;
      cv::absdiff(ii_cv, ii_mz, diff);
      
      double error = cv::sum(diff)[0];

      cv::absdiff(ii_cv, ii_mz_mt, diff);

      error += cv::sum(diff)[0];
      
      std::cout << "average mz is " << (elapsed_mz*1e-3)/max_iter << " ms\n";
      std::cout << "average mz (MT) is " << (elapsed_mz_mt*1e-3)/max_iter << " ms\n";
      std::cout << "average cv is " << (elapsed_cv*1e-3)/max_iter << " ms\n";
      std::cout << "error = " << error << "\n\n";
      
      if (error) {
        std::cerr << "something went wrong, quitting\n";
        if (isz.width <= 16 && isz.height <= 16) {
          std::cerr << ii_cv << "\n";
          std::cerr << ii_mz << "\n";
          std::cerr << ii_mz_mt << "\n";
        }
        exit(1);
      }

      image_u32_destroy(ii);
      image_u32_destroy(ii_mt);

    }
  }

  workerpool_destroy(wp);
  
}


void test_box() {

  workerpool_t* wp = workerpool_create(4);

  for (int i=0; img_sizes[i].width; ++i) {

    cv::Size isz(img_sizes[i]);

    for (int b=0; box_sizes[b]; ++b) {

      int bsz = box_sizes[b];

      Mat8uc1 orig(isz), cv_blur;
      
      cv::theRNG().fill(orig, cv::RNG::UNIFORM, 0, 255);
      
      image_u8_t src8 = cv2im8(orig);

      image_u8_t* iblur = 0;
      image_u8_t* iblur_mt = 0;
      
      int64_t elapsed_mz = 0, elapsed_mz_mt = 0, elapsed_cv = 0;

      std::cout << "box filtering size " << isz << " with width " << bsz << "\n";

      for (int iter=0; iter<max_iter; ++iter) {
        if (iblur) { image_u8_destroy(iblur); }
        int64_t start = utime_now();
        iblur = box_filter_border_replicate(&src8, bsz);
        elapsed_mz += utime_now() - start;
      }

      for (int iter=0; iter<max_iter; ++iter) {
        if (iblur_mt) { image_u8_destroy(iblur_mt); }
        int64_t start = utime_now();
        iblur_mt = box_filter_border_replicate_mt(&src8, bsz, wp);
        elapsed_mz_mt += utime_now() - start;
      }

      for (int iter=0; iter<max_iter; ++iter) {
        int64_t start = utime_now();
        cv::blur(orig, cv_blur, cv::Size(bsz, bsz), 
                 cv::Point(-1, -1), cv::BORDER_REPLICATE);
        elapsed_cv += utime_now() - start;
      }

      cv::Mat mz_blur = im2cv(iblur);
      cv::Mat mz_mt_blur = im2cv(iblur_mt);

      cv::Mat diff;
      
      cv::absdiff(mz_blur, cv_blur, diff);

      double error = cv::sum(diff)[0];

      cv::absdiff(mz_mt_blur, cv_blur, diff);

      error += cv::sum(diff)[0];
      
      std::cout << "average mz is " << (elapsed_mz*1e-3)/max_iter << " ms\n";
      std::cout << "average mz (MT) is " << (elapsed_mz_mt*1e-3)/max_iter << " ms\n";
      std::cout << "average cv is " << (elapsed_cv*1e-3)/max_iter << " ms\n";
      std::cout << "error = " << error << "\n\n";

      if (error) {
        std::cerr << "something went wrong, quitting\n";
        if (isz.width <= 16 && isz.height <= 16) {
          std::cerr << cv_blur << "\n";
          std::cerr << mz_blur << "\n";
          std::cerr << mz_mt_blur << "\n";
        }
        exit(1);
      }

      image_u8_destroy(iblur_mt);
      image_u8_destroy(iblur);

    }

  }

  workerpool_destroy(wp);

}

void test_thresh() {

  workerpool_t* wp = workerpool_create(4);

  for (int i=0; img_sizes[i].width; ++i) {

    cv::Size isz(img_sizes[i]);

    for (int b=0; box_sizes[b]; ++b) {

      int bsz = box_sizes[b];

      for (int t=0; thresholds[t] < 256; ++t) {

        int tau = thresholds[t];

        for (int inv=0; inv<2; ++inv) {

          Mat8uc1 orig(isz), cv_thresh;
      
          cv::theRNG().fill(orig, cv::RNG::UNIFORM, 0, 255);
      
          image_u8_t src8 = cv2im8(orig);

          image_u8_t* ithresh = 0;
          image_u8_t* ithresh_mt = 0;
      
          int64_t elapsed_mz = 0, elapsed_mz_mt = 0, elapsed_cv = 0;

          std::cout << "box thresholding size " << isz << " with width " << bsz << " and threshold " << tau << " and invert " << inv << "\n";

          for (int iter=0; iter<max_iter; ++iter) {
            if (ithresh) { image_u8_destroy(ithresh); }
            int64_t start = utime_now();
            ithresh = box_threshold(&src8, 255, inv, bsz, tau);
            elapsed_mz += utime_now() - start;
          }

          for (int iter=0; iter<max_iter; ++iter) {
            if (ithresh_mt) { image_u8_destroy(ithresh_mt); }
            int64_t start = utime_now();
            ithresh_mt = box_threshold_mt(&src8, 255, inv, bsz, tau, wp);
            elapsed_mz_mt += utime_now() - start;
          }

          for (int iter=0; iter<max_iter; ++iter) {
            int64_t start = utime_now();
            // void adaptiveThreshold(InputArray src, OutputArray dst, double maxValue, int adaptiveMethod, int thresholdType, int blockSize, double C)¶
            cv::adaptiveThreshold(orig, cv_thresh, 255, cv::ADAPTIVE_THRESH_MEAN_C, inv ? cv::THRESH_BINARY_INV : cv::THRESH_BINARY, bsz, tau);
            elapsed_cv += utime_now() - start;
          }

          cv::Mat mz_thresh = im2cv(ithresh);
          cv::Mat mz_mt_thresh = im2cv(ithresh_mt);

          cv::Mat diff;
      
          cv::absdiff(mz_thresh, cv_thresh, diff);

          double error = cv::sum(diff)[0];

          cv::absdiff(mz_mt_thresh, cv_thresh, diff);

          error += cv::sum(diff)[0];
      
          std::cout << "average mz is " << (elapsed_mz*1e-3)/max_iter << " ms\n";
          std::cout << "average mz (MT) is " << (elapsed_mz_mt*1e-3)/max_iter << " ms\n";
          std::cout << "average cv is " << (elapsed_cv*1e-3)/max_iter << " ms\n";
          std::cout << "error = " << error << "\n\n";

          if (error) {
            std::cerr << "something went wrong, quitting\n";
            if (isz.width <= 16 && isz.height <= 16) {
              std::cerr << cv_thresh << "\n";
              std::cerr << mz_thresh << "\n";
              std::cerr << mz_mt_thresh << "\n";
            }
            exit(1);
          }

          image_u8_destroy(ithresh_mt);
          image_u8_destroy(ithresh);

        }

      }

    }

  }

  workerpool_destroy(wp);

}


int main(int argc, char *argv[]) {
  
  test_integrate();
  
  test_box();

  test_thresh();

  return 0;

}
