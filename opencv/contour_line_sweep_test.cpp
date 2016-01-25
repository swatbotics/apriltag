#include "contour.h"
#include <stdio.h>
#include <vector>
#include <opencv2/opencv.hpp>

typedef struct string_image {
  int rows; 
  int cols;
  char buf[2048];
} string_image_t;

const string_image_t images[] = {

  { 1, 2,
    "XX" },

  { 3, 4,
    "xx x"
    "x xx"
    " xxx" },

  { 3, 3,
    "xxx"
    "x x"
    "xxx" },
  
  { 3, 5,
    "xxxxx"
    "x x x"
    "xxxxx" },

  { 3, 9,
    "xxxxxxxxx"
    "x x x x x"
    "xxxxxxxxx" },


  { 5, 5,
    "xxxxx"
    "x   x"
    "x x x"
    "x   x"
    "xxxxx" },

  { 3, 5,
    " xxx "
    "xx xx"
    " xxx " },

  { 3, 5,
    "xx xx"
    " xxx "
    "xx xx" },
  
  { 0, 0, "" }


};


int main(int argc, char** argv) {

  for (int k=0; images[k].rows; ++k) {
    
    const string_image_t* src = &(images[k]);

    image_u8_t* im = image_u8_create_alignment(src->cols, src->rows, 1);

    if (im->stride != im->width) {
      fprintf(stderr, "bad stride=%d (width=%d)\n", im->stride, im->width);
      exit(1);
    }

    int offs = 0;

    for (int i=0; i<src->rows; ++i) {
      for (int j=0; j<src->cols; ++j) {
        im->buf[offs] = (src->buf[offs] == ' ' ? 0 : 255);
        ++offs;
      }
    }

    zarray_t* contours = contour_line_sweep(im);

    printf("got %d contours:\n[\n", zarray_size(contours));
    
    for (int i=0; i<zarray_size(contours); ++i) {
      zarray_t* contour;
      zarray_get(contours, i, &contour);
      printf("  [ ");
      for (int j=0; j<zarray_size(contour); ++j) {
        const contour_point_t* pj;
        zarray_get_volatile(contour, j, &pj);
        printf("%s(%d, %d)", j ? ", " : "", (int)pj->x, (int)pj->y);
      }
      printf(" ]%s\n", i+1 == zarray_size(contours) ? "" : ",");
    }

    printf("]\n\n");

    //////////////////////////////////////////////////

    if (1) {

      int scl = 32;
    
      std::vector< std::vector<cv::Point> > cv_contours(zarray_size(contours));

      for (int i=0; i<zarray_size(contours); ++i) {
        zarray_t* contour;
        zarray_get(contours, i, &contour);
        cv_contours[i].resize(zarray_size(contour));
        for (int j=0; j<zarray_size(contour); ++j) {
          const contour_point_t* pj;
          zarray_get_volatile(contour, j, &pj);
          cv_contours[i][j] = cv::Point(pj->x, pj->y) * scl;
        }
      }


      std::vector< const cv::Point* > cv_ptrs(cv_contours.size());
      std::vector< int > cv_npts(cv_contours.size());
    
      for (size_t i=0; i<cv_contours.size(); ++i) {
        cv_ptrs[i] = &(cv_contours[i][0]);
        cv_npts[i] = cv_contours[i].size();
      }

      cv::Mat display = cv::Mat::zeros(scl*(im->height+2),
                                       scl*(im->width+2), CV_8UC3);

      cv::Point offset(scl, scl);

      if (cv_ptrs.size()) {
        cv::fillPoly(display, &cv_ptrs[0], &cv_npts[0], cv_ptrs.size(),
                     CV_RGB(255,255,255), 4, 0, offset);
      }

      for (size_t i=0; i<cv_contours.size(); ++i) {
        for (size_t j=0; j<cv_contours[i].size(); ++j) {
          size_t k = (j+1) % cv_contours[i].size();
          cv::Point pj = cv_contours[i][j] + offset;
          cv::Point pk = cv_contours[i][k] + offset;
          cv::Point d = pk - pj;
          // up: red, down: cyan
          // left: green, right: magenta
          cv::Scalar color;
          if (d.x < 0) {
            color = CV_RGB(0, 255, 0);
          } else if (d.y < 0) {
            color = CV_RGB(255, 0, 0);
          } else if (d.x > 0) {
            color = CV_RGB(255, 0, 255);
          } else {
            color = CV_RGB(0, 255, 255);
          }
          cv::line(display, pj, pk, color, 2);
        }
      }

      for (size_t i=0; i<cv_contours.size(); ++i) {
        for (size_t j=0; j<cv_contours[i].size(); ++j) {
          cv::Point pj = cv_contours[i][j] + offset;
          cv::circle(display, pj, 4, CV_RGB(0, 0, 0), -1);
          cv::circle(display, pj, 2, CV_RGB(255, 255, 255), -1);
        }
      }
    
      cv::imshow("win", display);
      cv::waitKey();

    }
    
    image_u8_destroy(im);

    contour_line_sweep_destroy(contours);
    
  }

  return 0;

}

