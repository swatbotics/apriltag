#ifndef _APRILTAG_QUAD_CONTOUR_H_
#define _APRILTAG_QUAD_CONTOUR_H_

#include "g2d.h"
#include "image_u8.h"
#include "zarray.h"
#include "apriltag.h"

#ifdef __cplusplus
extern "C" {
#endif

void apriltag_quad_contour_defaults(struct apriltag_quad_contour_params* qcp);
zarray_t* apriltag_quad_contour(apriltag_detector_t* td, image_u8_t* im);

#ifdef __cplusplus
}
#endif

#endif
