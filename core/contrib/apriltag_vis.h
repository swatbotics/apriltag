#ifndef _APRILTAG_VIS
#define _APRILTAG_VIS

#include "apriltag.h"

#ifdef __cplusplus
extern "C" {
#endif

image_u8_t* apriltag_vis_image(const apriltag_detection_t* detection);
matd_t* apriltag_vis_get_warp(const apriltag_detection_t* detection);

#ifdef __cplusplus
}
#endif

#endif
