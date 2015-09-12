#ifndef _BOX_H
#define _BOX_H

#include "image_u8.h"
#include "image_u32.h"

#ifdef __cplusplus
extern "C" {
#endif

// compute box filter on image
void box_filter(const image_u8_t* src, 
                image_u8_t* dst, 
                int sz);

void box_threshold(const image_u8_t* src,
                   image_u8_t* dst,
                   int max_value,
                   int invert,
                   int sz, 
                   int tau);

image_u32_t* integrate(const image_u8_t* img, int l);

#ifdef __cplusplus
}
#endif

#endif
