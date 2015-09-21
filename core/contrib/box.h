#ifndef _BOX_H
#define _BOX_H

#include "image_u8.h"
#include "image_u32.h"
#include "workerpool.h"

#ifdef __cplusplus
extern "C" {
#endif


image_u32_t* integrate_border_replicate(const image_u8_t* img, int l);

image_u8_t* box_filter_border_replicate(const image_u8_t* src, int sz);

image_u8_t* box_threshold(const image_u8_t* src,
                          int max_value,
                          int invert,
                          int sz, 
                          int tau);


image_u32_t* integrate_border_replicate_mt(const image_u8_t* img, int l,
                                           workerpool_t* wp);

image_u8_t* box_filter_border_replicate_mt(const image_u8_t* src, 
                                           int sz, workerpool_t* wp);

image_u8_t* box_threshold_mt(const image_u8_t* src,
                             int max_value,
                             int invert,
                             int sz, 
                             int tau,
                             workerpool_t* wp);

#ifdef __cplusplus
}
#endif

#endif
