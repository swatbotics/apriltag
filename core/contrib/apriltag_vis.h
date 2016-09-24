#ifndef _APRILTAG_VIS
#define _APRILTAG_VIS

#include "apriltag.h"

#ifdef __cplusplus
extern "C" {
#endif

image_u8_t* apriltag_vis_texture(const apriltag_detection_t* detection);
matd_t* apriltag_vis_warp(const apriltag_detection_t* detection);

void apriltag_vis_detection(const apriltag_detection_t* detection,
                            image_u8_t* image);

void apriltag_vis_detections(zarray_t* detections,
                             image_u8_t* image);

#ifdef __cplusplus
}
#endif

#endif
