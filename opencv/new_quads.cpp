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


void test_integrate() {

  cv::Size img_sizes[] = {
    cv::Size(4, 3),
    cv::Size(64, 48),
    cv::Size(640, 480),
    cv::Size(1280, 960),
    cv::Size(0,0),
  };

  int lsizes[] = {
    0, 2, 7, -1
  };

  for (int i=0; img_sizes[i].width; ++i) {
    
    for (int k=0; lsizes[k]>=0; ++k) {

      int l = lsizes[k];
    
      cv::Size isz = img_sizes[i];

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

      image_u32_t* ii = integrate(&src8, l);

      timeprofile_stamp(tp, "mz integral");
    
      cv::integral(src_b, ii_cv);

      timeprofile_stamp(tp, "cv integral");

      std::cout << "for size: " << isz << " with border " << l << "\n";

      timeprofile_display(tp);
      timeprofile_destroy(tp);

      ii_mz = im2cv(ii);

      Mat32sc1 diff;
      cv::absdiff(ii_cv, ii_mz, diff);

      double error = cv::sum(diff)[0];

      std::cout << "error = " << error << "\n\n";

      if (error) {
        std::cerr << "something went wrong, quitting\n";
        exit(1);
      }
    
    }

  }
  
  
  
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

      box_filter(&image, &image, bsz);

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

void test_xyw_fit() {

  cv::RNG& rng = cv::theRNG();

  for (int iter=0; iter<2; ++iter) {

    double theta = rng.uniform(0.0, 2.0*M_PI);
    double u[2] = { cos(theta), sin(theta) };

    if (iter == 0) {
      u[0] = 1.0;
      u[1] = 0.0;
    } else if (iter == 1) {
      u[0] = 0.0;
      u[1] = 1.0;
    }

    double p[2] = { rng.uniform(0.0, 10.0), rng.uniform(0.0, 10.0) };

    double l = rng.uniform(2.0, 10.0);

    xyw_moments_t m;
    memset(&m, 0, sizeof(m));

    for (int n=0; n<8; ++n) {

      double k = rng.uniform(0.0, l);
      double w = rng.uniform(1.0, 10.0);

      for (double sign=-1; sign<=1; sign+=2) {
        double x = p[0] + u[0]*k*sign;
        double y = p[1] + u[1]*k*sign;
        xyw_accum(&m, x, y, w);
      }
      
    }

    g2d_line_t line;
    line_init_from_xyw(&m, &line);

    double dp = u[0]*line.u[0] + u[1]*line.u[1];
    if (dp < 0) {
      dp = -dp;
      line.u[0] = -line.u[0];
      line.u[1] = -line.u[1];
    }

    printf("orig: u=(%f, %f), p=(%f, %f)\n",
           u[0], u[1], p[0], p[1]);

    printf("fit:  u=(%f, %f), p=(%f, %f), dp=%f\n\n",
           line.u[0], line.u[1], line.p[0], line.p[1], dp);

    if (dp < 1-1e-6) {
      fprintf(stderr, "fit failed!\n");
      exit(1);
    }
    
  }

  
}

int main(int argc, char *argv[]) {

  //test_outer();
  //return 0;

  //contour_test_outer();
  //return 0;

  /*
  test_integrate();
  test_box();
  return 0;
  */

  //test_xyw_fit();
  //return 0;

  getopt_t *getopt = getopt_create();

  getopt_add_bool(getopt, 'h', "help", 0, "Show this help");

  if (!getopt_parse(getopt, argc, argv, 1) || getopt_get_bool(getopt, "help")) {
    printf("Usage: %s [options] <input files>\n", argv[0]);
    getopt_do_usage(getopt);
    exit(0);
  }

  const zarray_t *inputs = getopt_get_extra_args(getopt);

  for (int input=0; input<zarray_size(inputs); ++input) {

    char *path;
    zarray_get(inputs, input, &path);

    cv::Mat orig = cv::imread(path);

    Mat8uc1 gray;

    if (orig.channels() == 3) {
      cv::cvtColor(orig, gray, cv::COLOR_RGB2GRAY);
    } else {
      orig.copyTo(gray);
    }

    image_u8_t* im = cv2im8_copy(gray);
    image_u8_t* t8 = image_u8_create(im->width, im->height);
    
    //////////////////////////////////////////////////////////////////////

    timeprofile_t* tp = timeprofile_create();

    //////////////////////////////////////////////////////////////////////

    box_threshold(im, t8, 255, 1, 15, 5);
    timeprofile_stamp(tp, "threshold");

    //////////////////////////////////////////////////////////////////////

    zarray_t* contours = contour_detect(t8);
    timeprofile_stamp(tp, "contour");

    //////////////////////////////////////////////////////////////////////
    // also parallelizable

    zarray_t* quads = quads_from_contours(im, contours);
    timeprofile_stamp(tp, "quads from contours");

    //////////////////////////////////////////////////////////////////////

    std::cout << "my time profile:\n";
    timeprofile_display(tp);
    timeprofile_destroy(tp);

    std::vector<cv::Point> points;
      
    /*

    cv::Mat contour_display = orig * 0.75;
    cv::Mat outer_display = orig * 0.75;

    for (int c=0; c<zarray_size(contours); ++c) {

      contour_info_t* ci;
      zarray_get_volatile(contours, c, &ci);

      if (ci->is_outer) {

        float ctr[2];
        float area = contour_area_centroid(ci->points, ctr);

        if (fabs(area) >= 64) {

          cv::Scalar color = random_color();

          polylines(contour_display, ci->points, color);

          zarray_t* bpoints = contour_outer_boundary(ci, 0, zarray_size(ci->points));

          polylines(outer_display, bpoints, color);


          char buf[1024];
          snprintf(buf, 1024, "%.1f", area);

          int bl;

          cv::Size s = getTextSize(buf, cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, &bl);

          ctr[0] -= s.width/2;
          ctr[1] += s.height/2;

          cv::putText(contour_display, buf, cv::Point(ctr[0], ctr[1]),
                      cv::FONT_HERSHEY_SIMPLEX, 0.4, CV_RGB(0,0,0), 3, CV_AA);

          cv::putText(contour_display, buf, cv::Point(ctr[0], ctr[1]),
                      cv::FONT_HERSHEY_SIMPLEX, 0.4, color, 1, CV_AA);

        }

      }

    }
    */

    cv::Mat arrow_display = orig * 0.75;

    for (int i=0; i<zarray_size(quads); ++i) {

      struct quad* qi;
      zarray_get_volatile(quads, i, &qi);

      fprintf(stderr, "p = {");
      for (int i=0; i<4; ++i) {
        fprintf(stderr, "%s(%.1f, %.1f)", i ? ", " : " ", qi->p[i][0], qi->p[i][1]);
      }
      fprintf(stderr, " }\n\n");

      cv::Point2f pts[4];
      for (int j=0; j<4; ++j) {
        pts[j] = cv::Point2f(qi->p[j][0], qi->p[j][1]);
      }

      cv::Scalar color = random_color();

      for (int j=0; j<4; ++j) {
        arrow(arrow_display, pts[j], pts[(j+1)&3], color);
      }

    }


    // filtering: ratio of orig area to hull area should be > 0.8 ?
    // hull area should be greater than # of pixels
    // orig perimiter shouldn't be too big or too small


    cv::imshow("win", orig);
    cv::waitKey();

    /*
    cv::imshow("win", im2cv(t8));
    cv::waitKey();

    cv::imshow("win", contour_display);
    cv::waitKey();
    
    cv::imshow("win", outer_display);
    cv::waitKey();
    */

    cv::imshow("win", arrow_display);
    cv::waitKey();

    zarray_destroy(quads);
    contour_destroy(contours);

    image_u8_destroy(t8);
    image_u8_destroy(im);

  }

  return 0;

}
