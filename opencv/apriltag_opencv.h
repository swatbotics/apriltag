#ifndef _APRILTAG_OPENCV_H_
#define _APRILTAG_OPENCV_H_

#include "apriltag.h"

#include <opencv2/core/core.hpp>

#include "image_u32.h"

typedef cv::Mat_<uint8_t> Mat8uc1;
typedef cv::Mat_<int32_t> Mat32sc1;
typedef cv::Mat_<double> Mat64fc1;

inline Mat8uc1 im2cv(image_u8_t* im) {
  if (im->stride < im->width) {
    fprintf(stderr,"we have issues!\n");
    exit(1);
  }
  Mat8uc1 orig(im->height, im->stride, im->buf);
  return orig.colRange(0, im->width);
}

inline Mat32sc1 im2cv(image_u32_t* im) {
  if (im->stride < im->width) {
    fprintf(stderr,"we have issues!\n");
    exit(1);
  }
  Mat32sc1 orig(im->height, im->stride, (int32_t*)im->buf);
  return orig.colRange(0, im->width);
}


inline image_u8_t* cv2im8_copy(cv::Mat m) {
  
  if (!m.isContinuous() || m.type() != CV_8UC1) {
    fprintf(stderr, "not continuous 8UC1\n");
    exit(1);
  }

  image_u8_t* im = image_u8_create(m.cols, m.rows);

  cv::Mat dst = im2cv(im);
  m.copyTo(dst);

  return im;

}


inline image_u32_t* cv2im32_copy(cv::Mat m) {
  
  if (!m.isContinuous() || m.type() != CV_32SC1) {
    fprintf(stderr, "not continuous 32SC1\n");
    exit(1);
  }

  image_u32_t* im = image_u32_create(m.cols, m.rows);

  cv::Mat dst = im2cv(im);
  m.copyTo(dst);

  return im;

}


inline image_u8_t cv2im8(cv::Mat m) {
  
  if (!m.isContinuous() || m.type() != CV_8UC1) {
    fprintf(stderr, "not continuous 8UC1\n");
    exit(1);
  }
  
  image_u8_t tmp = { .width = m.cols,
                     .height = m.rows,
                     .stride = m.cols,
                     .buf = (uint8_t*)m.data };

  return tmp;

}

inline image_u32_t cv2im32(cv::Mat m) {
  
  if (!m.isContinuous() || m.type() != CV_32SC1) {
    fprintf(stderr, "not continuous 32SC1\n");
    exit(1);
  }

  image_u32_t tmp = { .width = m.cols,
                      .height = m.rows,
                      .stride = m.cols,
                      .buf = (uint32_t*)m.data };

  return tmp;

}


Mat8uc1 makeImage(const apriltag_detection_t* det);

Mat64fc1 getWarp(const apriltag_detection_t* detection);

cv::Mat detectionImage(const apriltag_detection_t* detection,
                       const cv::Size& size,
                       int type,
                       const cv::Scalar& bgcolor=cv::Scalar(0,0,0,0));

#endif
