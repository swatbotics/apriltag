#include "apriltag_vis.h"
#include "homography.h"
#include <math.h>

image_u8_t* apriltag_vis_texture(const apriltag_detection_t* det) {
  
  const apriltag_family_t* family = det->family;
  
  const uint32_t wb = 1;
  const uint32_t bb = family->black_border;
  const uint32_t tb = wb + bb;

  const uint32_t d = family->d;

  uint32_t size = 2*tb + d;

  const uint8_t white = 255, black = 0;
  
  image_u8_t* rval = image_u8_create(size, size);
  uint8_t* rowptr = rval->buf;

  // borders first
  for (uint32_t y=0; y<size; ++y) {
    for (uint32_t x=0; x<size; ++x) {
      if (y < wb || y+wb >= size || x < wb || x+wb >= size) {
        rowptr[x] = white;
      } else {
        rowptr[x] = black;
      }
    }
    rowptr += rval->stride;
  }

  uint64_t v = family->codes[det->id];

  uint64_t bits = d*d;

  // then interior
  rowptr = rval->buf + tb*rval->stride + tb;
  
  for (uint32_t y=0; y<d; ++y) {
    for (uint32_t x=0; x<d; ++x) {
      if ((v&((uint64_t)(1)<<(uint64_t)(bits-1)))!=0) {
        rowptr[x] = white;
      } else {
        rowptr[x] = black;
      }
      v = v<<(uint64_t)(1);
    }
    rowptr += rval->stride;
  }

  return rval;

}

matd_t* apriltag_vis_warp(const apriltag_detection_t* detection) {

  double d = detection->family->d;
  double bb = detection->family->black_border;
  double wb = 1;

  double tb = 2*wb - bb;
  
  double sz = d + 2*bb;

  double Sdata[9] = {
    1, 0, 0,
    0, -1, 0,
    0, 0, 1
  };

  double Cdata[9] = {
    2.0/sz, 0.0,  -1.0 - tb/sz,
    0.0, -2.0/sz,  1.0 + tb/sz,
    0, 0, 1
  };

  matd_t* S = matd_create_data(3, 3, Sdata);
  matd_t* C = matd_create_data(3, 3, Cdata);

  matd_t* H = detection->H;

  matd_t* result = matd_op("M*M*M", H, S, C);

  matd_destroy(S);
  matd_destroy(C);

  return result;
  
}

static inline int imin(int x, int y) {
  return x < y ? x : y;
}

static inline int imax(int x, int y) {
  return x > y ? x : y;
}

void apriltag_vis_detection(const apriltag_detection_t* detection,
                            image_u8_t* image) {

  // get points and homography
  int x0 = image->width;
  int x1 = 0;

  int y0 = image->height;
  int y1 = 0;

  matd_t* warp = apriltag_vis_warp(detection);

  image_u8_t* texture = apriltag_vis_texture(detection);
  int size = texture->width;

  int points[4][2] = {
    { -1, -1 },
    { -1, size },
    { size, size },
    { size, -1 }
  };
  
  for (int i=0; i<4; ++i) {
    double xi, yi;
    homography_project(warp, points[i][0], points[i][1], &xi, &yi);
    x0 = imin(x0, floor(xi));
    x1 = imax(x1, ceil(xi));
    y0 = imin(y0, floor(yi));
    y1 = imax(y1, ceil(yi));
  }

  x0 = imax(x0, 0);
  x1 = imin(x1, image->width);

  y0 = imax(y0, 0);
  y1 = imin(y1, image->height);

  matd_t* warp_inv = matd_inverse(warp);
  
  uint8_t* rowptr = image->buf + y0*image->stride;
  
  for (int y=y0; y<y1; ++y) {
    for (int x=x0; x<x1; ++x) {
      double ox, oy;
      homography_project(warp_inv, x+0.5, y+0.5, &ox, &oy);
      int ix = (int)floor(ox+0.5);
      int iy = (int)floor(oy+0.5);
      if (ix >= 0 && ix < size && iy >= 0 && iy < size) {
        rowptr[x] = texture->buf[iy*texture->stride + ix];
      }
    }
    rowptr += image->stride;
  }

  image_u8_destroy(texture);
  matd_destroy(warp);
  matd_destroy(warp_inv);

}

void apriltag_vis_detections(zarray_t* detections,
                             image_u8_t* image) {

  for (int i = 0; i < zarray_size(detections); i++) {
    apriltag_detection_t *det;
    zarray_get(detections, i, &det);
    apriltag_vis_detection(det, image);
  }
  
  
}
