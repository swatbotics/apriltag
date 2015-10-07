#ifndef _CONTOUR_H
#define _CONTOUR_H

#include "image_u8.h"
#include "zarray.h"
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct contour_point {
  uint32_t x, y;
} contour_point_t;

typedef struct contour_info {
  int parent;   // parent border index
  int is_outer; // 1 if outer, 0 if inner
  zarray_t* points; // make points
} contour_info_t;

zarray_t* contour_detect(const image_u8_t* im);

void contour_destroy(zarray_t* contours);

// take array of contour_point_t as input
// returns array of contour_point_t, caller must deallocate
zarray_t* contour_convex_hull(const zarray_t* orig_points);

float contour_area_centroid(const zarray_t* points,
                            float centroid[2]);

// returns array of contour_point_t, caller must deallocate
// ok for [start, start+count) to wrap around end of contour
zarray_t* contour_outer_boundary(const contour_info_t* c,
                                 int start,
                                 int count);

// returns zarray_t of zarray_t of contour_point
zarray_t* contour_line_sweep(const image_u8_t* im);

void contour_line_sweep_destroy(zarray_t* contours);

#ifdef __cplusplus
}
#endif

#endif
