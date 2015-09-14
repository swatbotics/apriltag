#ifndef _APRILTAG_QUAD_CONTOUR_H_
#define _APRILTAG_QUAD_CONTOUR_H_

#include "g2d.h"
#include "image_u8.h"
#include "zarray.h"
#include "apriltag.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct xyw_moments {
  double n;
  double mX;
  double mY;
  double mXX;
  double mYY;
  double mXY;
} xyw_moments_t;

inline void xyw_accum(xyw_moments_t* m,
                      double x, double y, double w) {

  //printf("%f %f %f\n", x, y, w);
  
  m->n += w;

  m->mX += x*w;
  m->mY += y*w;
  
  m->mXX += x*x*w;
  m->mYY += y*y*w;
  m->mXY += x*y*w;
  
}

void line_init_from_xyw(const xyw_moments_t* m, g2d_line_t* line);

void apriltag_quad_contour_defaults(struct apriltag_quad_contour_params* qcp);

zarray_t* apriltag_quad_contour(apriltag_detector_t* td,
                                image_u8_t* im);

#ifdef __cplusplus
}
#endif

#endif
