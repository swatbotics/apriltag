#include "apriltag_vis.h"

image_u8_t* apriltag_vis_image(const apriltag_detection_t* det) {
  
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

matd_t* apriltag_vis_get_warp(const apriltag_detection_t* detection) {

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

  /*
  Mat64fc1 S = Mat64fc1::eye(3,3);
  S(1,1) = -1;
  
  Mat64fc1 H(3,3, detection->H->data);

  Mat64fc1 C = Mat64fc1::eye(3,3);

  C(0,0) = 2.0/sz;
  C(0,2) = -1.0 - tb/sz;
  C(1,1) = -2.0/sz;
  C(1,2) = 1.0 + tb/sz;

  return H * S * C;
  */

  return result;

  
}
