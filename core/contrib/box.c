#include "box.h"
#include "image_u32.h"
#include <alloca.h>
#include <stdlib.h>
#include <string.h>

/*
#ifdef NDEBUG
#undef NDEBUG
#endif
*/

#include <assert.h>
#include <stdio.h>

typedef struct cbuf {

  int width; // allocated size of data
  int count; // # currently occupied slots
  int tail; // one past end of data
  uint32_t total; // running total
  uint8_t* data;
  
} cbuf_t;

void cbuf_push(cbuf_t* c, uint8_t x) {

  assert(c->count < c->width);
  c->data[c->tail] = x;
  c->total += x;
  ++c->count;
  
  if (++c->tail == c->width) {
    c->tail = 0;
  }

  //printf("after pushing %d, count is %d and total is %d\n", (int)x, c->count, c->total);

}

void cbuf_pop(cbuf_t* c) {

  assert(c->count);

  int idx = c->tail - c->count;
  if (idx < 0) { idx += c->width; }

  assert(idx >= 0 && idx < c->count );
  c->total -= c->data[idx];
  --c->count;

}


void bfrc(uint8_t* data, int count, int skip, int sz) {

  int l = sz/2;
  sz = 2*l + 1; // ensure odd!

  cbuf_t c = { sz, 0, 0, 0, 0 };
  c.data = alloca(sz);

  int k;

  // left pad
  for (k=0; k<=l; ++k) { 
    cbuf_push(&c, *data); 
  }

  assert(c.total == *data * (l+1));
  assert(c.count == l+1);

  uint8_t* edge = data;
  uint8_t* mid = data;
  uint8_t* last_data = data + (count-1)*skip;
  
  for (k=1; k<l; ++k) { 
    if (edge != last_data) { 
      edge += skip;
    }
    cbuf_push(&c, *edge);
  }

  assert(c.count == sz-1);

  for (k=0; k<count; ++k) {

    assert(c.count == sz-1);

    if (edge != last_data) { 
      edge += skip;
    }
    cbuf_push(&c, *edge);
    assert(c.count == sz);

    uint32_t avg = (c.total+l)/sz;

    *mid = avg;

    cbuf_pop(&c);

    mid += skip;

  }

  assert(c.count == sz-1);
  assert(mid == last_data + skip);
  assert(edge == last_data);


}

/*
static inline int imin(int a, int b) { return a < b ? a : b; }
static inline int imax(int a, int b) { return a > b ? a : b; }

static inline int ilookup(const image_u32_t* iimg,
                          int x, int y) {

  x = imax(0, imin(x, iimg->width-1));
  y = imax(0, imin(y, iimg->height-1));

  return iimg->buf[iimg->stride*y + x];

}
*/

image_u32_t* integrate(const image_u8_t* img, int l) {

  image_u32_t tmp;
  tmp.width = img->width+2*l+1;
  tmp.height = img->height+2*l+1;
  tmp.stride = tmp.width;


  if (tmp.stride & 0xf) {
    tmp.stride += 16 - (tmp.stride & 0xf);
  }

  tmp.buf = (uint32_t*)malloc(tmp.height * tmp.stride * sizeof(uint32_t));

  memset(tmp.buf, 0, tmp.width*sizeof(uint32_t));
  image_u32_t* iimg = malloc(sizeof(image_u32_t));
  memcpy(iimg, &tmp, sizeof(image_u32_t));

  uint32_t* dst = iimg->buf + iimg->stride;
  const uint8_t* src = img->buf;

  int srcrem = img->stride - img->width;
  int dstrem = iimg->stride - iimg->width;
  int o10 = -1;
  int o01 = -iimg->stride;
  int o11 = o10 + o01;

  //////////////////////////////////////////////////////////////////////
  // TOP PADDING

  for (int y=0; y<l; ++y) {

    *dst++ = 0;

    const uint8_t* psrc = img->buf;

    // left padding
    for (int x=0; x<l; ++x) {
      *(dst) = (*psrc) + dst[o10] + dst[o01] - dst[o11];
      ++dst;
    }

    // main row
    for (int x=0; x<img->width; ++x) {
      *(dst) = (*psrc++) + dst[o10] + dst[o01] - dst[o11];
      ++dst;
    }

    // right padding
    for (int x=0; x<l; ++x) {
      *(dst) = psrc[-1] + dst[o10] + dst[o01] - dst[o11];
      ++dst;
    }

    dst += dstrem;

  }

  //////////////////////////////////////////////////////////////////////
  // MIDDLE ROWS

  for (int y=0; y<img->height; ++y) {

    *dst++ = 0;

    // left padding
    for (int x=0; x<l; ++x) {
      *(dst) = (*src) + dst[o10] + dst[o01] - dst[o11];
      ++dst;
    }

    // main row
    for (int x=0; x<img->width; ++x) {
      *(dst) = (*src++) + dst[o10] + dst[o01] - dst[o11];
      ++dst;
    }

    // right padding
    for (int x=0; x<l; ++x) {
      *(dst) = src[-1] + dst[o10] + dst[o01] - dst[o11];
      ++dst;
    }

    src += srcrem;
    dst += dstrem;

  }
  
  //////////////////////////////////////////////////////////////////////
  // BOTTOM PADDING

  const uint8_t* lastrow = img->buf + (img->height-1)*img->stride;

  for (int y=0; y<l; ++y) {

    *dst++ = 0;

    const uint8_t* psrc = lastrow;

    // left padding
    for (int x=0; x<l; ++x) {
      *(dst) = (*psrc) + dst[o10] + dst[o01] - dst[o11];
      ++dst;
    }

    // main row
    for (int x=0; x<img->width; ++x) {
      *(dst) = (*psrc++) + dst[o10] + dst[o01] - dst[o11];
      ++dst;
    }

    // right padding
    for (int x=0; x<l; ++x) {
      *(dst) = psrc[-1] + dst[o10] + dst[o01] - dst[o11];
      ++dst;
    }

    dst += dstrem;

  }

  return iimg;


}



void box_filter(const image_u8_t* src_img, 
                image_u8_t* dst_img,
                int sz) {

  if (!(src_img->width == dst_img->width &&
        src_img->height == dst_img->height)) {
    fprintf(stderr, "%s:%d: image size mismatch in %s\n", 
            __FILE__, __LINE__, __FUNCTION__);
    exit(1);
  }

  int l = sz/2;
  sz = 2*l+1;

  image_u32_t* tmp_img = integrate(src_img, l);

  uint8_t* dst = dst_img->buf;
  int s2 = sz*sz;
  int s22 = s2/2; // for rounding?

  const uint32_t* tmp = tmp_img->buf;

  int ob = tmp_img->stride*sz;
  int od = ob + sz;

  int tmprem = tmp_img->stride - dst_img->width;
  int dstrem = dst_img->stride - dst_img->width;

  for (int y=0; y<dst_img->height; ++y) {
    for (int x=0; x<dst_img->width; ++x) {
      int t = (tmp[od] - tmp[ob] - tmp[sz] + *tmp + s22)/s2;
      *dst++ = t;
      ++tmp;
    }
    dst += dstrem;
    tmp += tmprem;
  }

  image_u32_destroy(tmp_img);

}


void box_threshold(const image_u8_t* src_img, 
                   image_u8_t* dst_img,
                   int max_value, 
                   int invert, 
                   int sz, 
                   int tau) {

  if (!(src_img->width == dst_img->width &&
        src_img->height == dst_img->height)) {
    fprintf(stderr, "%s:%d: image size mismatch in %s\n", 
            __FILE__, __LINE__, __FUNCTION__);
    exit(1);
  }


  int gt = invert ? 0 : max_value;
  int lt = max_value - gt;

#if 0

  image_u8_t* tmp_img = image_u8_create_alignment(src_img->width,
                                                  src_img->height,
                                                  1);

  box_filter(src_img, tmp_img, sz);


  const uint8_t* s = src_img->buf;
  const uint8_t* t = tmp_img->buf;
  uint8_t* res = dst_img->buf;

  for (int y=0; y<src_img->height; ++y) {
    for (int x=0; x<src_img->width; ++x) {
      int sx = s[x];
      int tx = t[x] - tau;
      res[x] = sx > tx ? gt : lt;
    }
    s += src_img->stride;
    t += tmp_img->stride;
    res += dst_img->stride;
  }
                           
  image_u8_destroy(tmp_img);

#else

  int l = sz/2;
  sz = 2*l+1;

  image_u32_t* tmp_img = integrate(src_img, l);

  uint8_t* dst = dst_img->buf;
  int s2 = sz*sz;
  int s22 = s2/2; // for rounding?

  const uint32_t* tmp = tmp_img->buf;
  const uint8_t* src = src_img->buf;

  int ob = tmp_img->stride*sz;
  int od = ob + sz;

  int tmprem = tmp_img->stride - dst_img->width;
  int dstrem = dst_img->stride - dst_img->width;
  int srcrem = src_img->stride - dst_img->width;

  for (int y=0; y<dst_img->height; ++y) {
    for (int x=0; x<dst_img->width; ++x) {
      int t = (tmp[od] - tmp[ob] - tmp[sz] + *tmp + s22)/s2 - tau;
      int s = *src++;
      *dst++ = s > t ? gt : lt;
      ++tmp;
    }
    src += srcrem;
    dst += dstrem;
    tmp += tmprem;
  }

  image_u32_destroy(tmp_img);

#endif

}
