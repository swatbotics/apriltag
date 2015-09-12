#include "apriltag_opencv.h"
#include "getopt.h"
#include "contour.h"
#include "box.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <stdio.h>

               
cv::Scalar random_color() {
  
  cv::RNG& rng = cv::theRNG();
  
  cv::Scalar color = CV_RGB(rng.uniform(127, 256),
                            rng.uniform(127, 256),
                            rng.uniform(127, 256));

  color[rng.uniform(0,3)] = 0;

  return color;

}

cv::Point scale_point(int x, int y, int u) {
  int b = u/2;
  return cv::Point(x*u + b, y*u + b);
}

void contour_to_points(const zarray_t* cpoints,
                       std::vector<cv::Point>& points,
                       int upscale=1) {

  points.clear();

  for (int i=0; i<zarray_size(cpoints); ++i) {
    const contour_point_t* p;
    zarray_get_volatile(cpoints, i, &p);
    points.push_back(scale_point(p->x, p->y, upscale));
  }

}

void polylines(cv::Mat image,
               const zarray_t* cpoints,
               const cv::Scalar& color,
               bool closed=true,
               int u=1) {

  std::vector<cv::Point> points;
  
  contour_to_points(cpoints, points, u);
  
  //contour_to_points(ci->points, points);
  const cv::Point* p0 = &(points[0]);
  int npts = points.size();
    
  cv::polylines(image, &p0, &npts, 1, closed, color, 1);

  if (u>1) {
    for (size_t i=0; i<points.size(); ++i) {
      cv::circle(image, points[i], u/4+1, color, -1);
    }
  }

  
}

void debug_print(const image_u8_t* im) {

  const uint8_t* rowptr = im->buf;

  for (int y=0; y<im->height; ++y) {
    printf("[ ");
    for (int x=0; x<im->width; ++x) {
      printf("%3d ", (int)rowptr[x]);
    }
    printf("]\n");
    rowptr += im->stride;
  }

}



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


float furthest_point(const zarray_t* points, 
                     const float q[2],
                     float p[2],
                     int* idx) {

  float max_r2 = -1;

  // get furthest from centroid
  for (int i=0; i<zarray_size(points); ++i) {

    const contour_point_t* pi;
    zarray_get_volatile(points, i, &pi);

    float xi = pi->x;
    float yi = pi->y;

    float dx = xi - q[0];
    float dy = yi - q[1];

    float r2 = dx*dx + dy*dy;

    if (r2 > max_r2) {
      max_r2 = r2;
      p[0] = xi;
      p[1] = yi;
      *idx = i;
    }

  }

  return max_r2;

}

float furthest_along(const zarray_t* points, 
                     const float q[2],
                     float p[2],
                     int* idx) {

  float max_d = -1;

  // get furthest from centroid
  for (int i=0; i<zarray_size(points); ++i) {
    const contour_point_t* pi;
    zarray_get_volatile(points, i, &pi);
    float xi = pi->x;
    float yi = pi->y;
    float di = q[0]*xi + q[1]*yi;
    if (i==0 || di > max_d) {
      max_d = di;
      p[0] = xi;
      p[1] = yi;
      *idx = i;
    }
  }

  return max_d;

}



// for easy2.png, orig apriltags identifies corners at (basically):
// 
//   p = { (36, 19), (59, 19), (59, 42), (36, 42) }
//
// which implies a CW ordering as seen by some viewer of the image:
// (TL, TR, BR, BL) but math-wise, it's CCW because underlying coord
// sys is left-handed)
//
// my program emits
//
//   p = { [36, 19], [59, 19], [59, 42], [36, 42] }
//
// which is great. only caveat is winding order for quad is 
// different than winding order for the contour. NBD.
//
zarray_t* extract_rough_quads(const zarray_t* contours) {

  zarray_t* quads = zarray_create(sizeof(struct quad));

  for (int c=0; c<zarray_size(contours); ++c) {

    contour_info_t* ci;
    zarray_get_volatile(contours, c, &ci);

    if (ci->is_outer && zarray_size(ci->points) >= 32) {

      float ctr[2];
      float area = contour_area_centroid(ci->points, ctr);

      if (fabs(area) >= 64) {

        struct quad q;

        float* p0 = q.p[0];
        float* p1 = q.p[1];
        float* p2 = q.p[2];
        float* p3 = q.p[3];

        int idx[4];

        furthest_point(ci->points, ctr, p0, idx+0);
        float l2 = furthest_point(ci->points, p0, p2, idx+2);
        float l = sqrt(l2);

        float v[2] = { p2[1] - p0[1], 
                       p0[0] - p2[0] };

        v[0] /= l;
        v[1] /= l;

        float v1 = furthest_along(ci->points, v, p1, idx+1);

        v[0] = -v[0];
        v[1] = -v[1];

        float v3 = -furthest_along(ci->points, v, p3, idx+3);

        float w = fabs(v1-v3);
        float a = w/l;

        // check aspect ratio
        if (a > 0.4) {
          // TODO: check "quad defect" (convex defect)?
          // TODO: reject triangles (make sure 4 points unique!)
          zarray_add(quads, &q);

          //int n = zarray_size(ci->points);
          //zarray_t* outer = contour_outer_boundary(ci, -1, -1);
          //zarray_destroy(outer);

        }

      }

      
    }

  }

  return quads;

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

      zarray_t* outer = contour_outer_boundary(ci, -1, -1);

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


extern "C" {
void contour_test_outer();
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

    image_u8_t* t8 = cv2im8_copy(gray);
    
    //////////////////////////////////////////////////////////////////////

    timeprofile_t* tp = timeprofile_create();

    //////////////////////////////////////////////////////////////////////

    box_threshold(t8, t8, 255, 1, 15, 10);
    timeprofile_stamp(tp, "threshold");

    //////////////////////////////////////////////////////////////////////

    zarray_t* contours = contour_detect(t8);
    timeprofile_stamp(tp, "contour");

    //////////////////////////////////////////////////////////////////////
    // convex hulling could be parallized easily (OTOH it's fast)
    // don't seem to need it to get reasonable quads
    // OTOH convexity defect is a good rejection?

    /*
    for (int c=0; c<zarray_size(contours); ++c) {
      contour_info_t* ci;
      zarray_get_volatile(contours, c, &ci);
      if (ci->is_outer) {
        zarray_t* cvx_points = contour_convex_hull(ci->points);
        zarray_destroy(ci->points);
        ci->points = cvx_points;
      }
    }

    timeprofile_stamp(tp, "cvx hull");
    */

    //////////////////////////////////////////////////////////////////////
    // also parallelizable

    zarray_t* quads = extract_rough_quads(contours);

    timeprofile_stamp(tp, "rough quads");

    //////////////////////////////////////////////////////////////////////

    std::cout << "my time profile:\n";
    timeprofile_display(tp);
    timeprofile_destroy(tp);

    std::vector<cv::Point> points;
      
    cv::Mat display1 = orig * 0.25;
    cv::Mat display1b = display1.clone();
    cv::Mat display2 = display1.clone();

    for (int c=0; c<zarray_size(contours); ++c) {

      contour_info_t* ci;
      zarray_get_volatile(contours, c, &ci);

      if (ci->is_outer) {

        float ctr[2];
        float area = contour_area_centroid(ci->points, ctr);

        if (fabs(area) >= 64) {

          cv::Scalar color = random_color();

          polylines(display1, ci->points, color);

          zarray_t* bpoints = contour_outer_boundary(ci, 0, zarray_size(ci->points));

          polylines(display1b, bpoints, color);


          char buf[1024];
          snprintf(buf, 1024, "%.1f", area);

          int bl;

          cv::Size s = getTextSize(buf, cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, &bl);

          ctr[0] -= s.width/2;
          ctr[1] += s.height/2;

          cv::putText(display1, buf, cv::Point(ctr[0], ctr[1]),
                      cv::FONT_HERSHEY_SIMPLEX, 0.4, CV_RGB(0,0,0), 3, CV_AA);

          cv::putText(display1, buf, cv::Point(ctr[0], ctr[1]),
                      cv::FONT_HERSHEY_SIMPLEX, 0.4, color, 1, CV_AA);

        }

      }

    }

    for (int i=0; i<zarray_size(quads); ++i) {

      struct quad* qi;
      zarray_get_volatile(quads, i, &qi);
      cv::Point pts[4];

      //std::cout << "p = { ";
      for (int j=0; j<4; ++j) {
        pts[j] = cv::Point(qi->p[j][0], qi->p[j][1]);
        //std::cout << (j ? ", " : "") << pts[j];
      }
      //std::cout << " }\n";

      const cv::Point* cpts = pts;
      int npts = 4;

      cv::RNG& rng = cv::theRNG();
      cv::Scalar color = CV_RGB(rng.uniform(127, 256),
                                rng.uniform(127, 256),
                                rng.uniform(127, 256));

      cv::polylines(display2, &cpts, &npts, 1, true, color, 1);

    }

    contour_destroy(contours);
    zarray_destroy(quads);

    // filtering: ratio of orig area to hull area should be > 0.8 ?
    // hull area should be greater than # of pixels
    // orig perimiter shouldn't be too big or too small


    cv::imshow("win", orig);
    cv::waitKey();

    Mat8uc1 thresh = im2cv(t8);

    cv::imshow("win", thresh);
    cv::waitKey();

    cv::imshow("win", display1);
    cv::waitKey();
    
    cv::imshow("win", display1b);
    cv::waitKey();

    cv::imshow("win", display2);
    cv::waitKey();

    image_u8_destroy(t8);

  }

  return 0;

}
