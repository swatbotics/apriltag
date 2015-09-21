#include "box.h"
#include "image_u32.h"
#include <alloca.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <stdio.h>
#include <pthread.h>
#include "zarray.h"

image_u32_t* aligned_image_64bit(int width, int height) {

  int stride = width;

  if (stride & 0xf) {
    stride += 16 - (stride & 0xf);
  }

  void* vptr = 0;

  size_t size = height * stride * sizeof(uint32_t);
  int status = posix_memalign(&vptr, 16, size);
  
  if (status != 0 || !vptr) {
    return NULL;
  }

  image_u32_t* img = (image_u32_t*)malloc(sizeof(image_u32_t));
  
  img->width = width;
  img->height = height;
  img->stride = stride;
  img->buf = vptr;

  return img;
  
}

enum {
  INTEGRATE_BLOCK_SIZE = 16
};

static inline void integrate_block_center(const uint8_t* src, int sstride,
                                          uint32_t* dst, int dstride) {

  for (int y=0; y<INTEGRATE_BLOCK_SIZE; ++y) {
    const uint8_t* s = src;
    uint32_t* d = dst;
    for (int x=0; x<INTEGRATE_BLOCK_SIZE; ++x) {
      d[dstride+1] = *s++ + d[dstride] + d[1] - d[0];
      ++d;
    }
    src += sstride;
    dst += dstride;
  }

}

static inline void integrate_block(const uint8_t* src, int sstep, int sstride,
                                   uint32_t* dst, int dstride,
                                   int nx, int ny) {


  for (int y=0; y<ny; ++y) {
    const uint8_t* s = src;
    uint32_t* d = dst;
    for (int x=0; x<nx; ++x) {
      d[dstride+1] = *s + d[dstride] + d[1] - d[0];
      ++d;
      s += sstep;
    }
    src += sstride;
    dst += dstride;
  }

}
                                  
                                  
                                  

image_u32_t* integrate_border_replicate(const image_u8_t* img, int l) {

  image_u32_t* iimg = aligned_image_64bit(img->width + 2*l + 1,
                                          img->height + 2*l + 1);

  // zero out first line
  memset(iimg->buf, 0, sizeof(uint32_t)*iimg->width);
  uint32_t* dl = iimg->buf + iimg->stride;
  
  // zero out first row of each line
  for (int y=1; y<iimg->height; ++y) {
    *dl = 0 ;
    dl += iimg->stride;
  }
  
  const uint8_t* src = img->buf;

  uint32_t* dst = iimg->buf;
  int ds = iimg->stride;
  int ss = img->stride;

  int W = img->width, w = W-1, H = img->height, h = H-1;
    
  //////////////////////////////////////////////////////////////////////
  // TOP PADDING

  integrate_block(src, 0, 0, dst, ds, l, l);

  dst += l;

  integrate_block(src, 1, 0, dst, ds, W, l);

  dst += W;

  integrate_block(src + w, 0, 0, dst, ds, l, l);

  //////////////////////////////////////////////////////////////////////
  // MIDDLE ROWS

  dst = iimg->buf + ds*l;

  integrate_block(src, 0, ss, dst, ds,l, H);

  dst += l;

  integrate_block(src, 1, ss, dst, ds, W, H);

  dst += W;

  integrate_block(src + w, 0, ss, dst, ds, l, H);
  
  //////////////////////////////////////////////////////////////////////
  // BOTTOM PADDING

  dst = iimg->buf + ds*(l + H);

  integrate_block(src + h*ss, 0, 0, dst, ds, l, l);

  dst += l;

  integrate_block(src + h*ss, 1, 0, dst, ds, W, l);

  dst += W;

  integrate_block(src + h*ss + w, 0, 0, dst, ds, l, l);
  
  return iimg;


}

/*
static inline
uint32_t iimg_lookup_with_border_center(const image_u32_t* iimg,
                                        int sz, int i, int j) {

  assert( i >= -sz && j >= -sz && i < iimg->height+sz && j < iimg->width+sz );

  uint32_t* b = iimg->buf;
  int s = iimg->stride;

  uint32_t s11 = b[iimg->stride + 1];
  uint32_t sij = b[s*i+j];
  uint32_t si1 = b[s*i+1];
  uint32_t s1j = b[s + j];
  return sij + sz*(sz*s11 + si1 + s1j);
  
}


static inline
uint32_t iimg_lookup_with_border(const image_u32_t* iimg,
                                 int sz, int i, int j) {

  assert( i >= -sz && j >= -sz && i < iimg->height+sz && j < iimg->width+sz );

  int code = ( ((i < 0)<<3) | ((i >= iimg->height)<<2) |
               ((j >= iimg->width)<<1) | (j < 0) );

  uint32_t* b = iimg->buf;
  int s = iimg->stride;

  uint32_t s11 = b[iimg->stride + 1];

  int H = iimg->height-1, h = H-1, W = iimg->width-1, w = W-1;

  switch (code) {
  case 0: { // middle
    uint32_t sij = b[s*i+j];
    uint32_t si1 = b[s*i+1];
    uint32_t s1j = b[s + j];
    return sij + sz*(sz*s11 + si1 + s1j);
  }
  case 9: { // above left
    i += sz;
    j += sz;
    return s11 * i * j;
  }
  case 8: { // above
    i += sz;
    uint32_t s1j = b[s + j];
    return (s1j + s11*sz)*i;
  }
  case 1: { // left
    j += sz;
    uint32_t si1 = b[s*i+1];
    return (si1 + s11*sz)*j;
  }
  case 10: { // above right
    uint32_t s1W = b[s + W];
    uint32_t s1w = b[s + w];
    j -= W;
    i += sz;
    return (s1W + (s1W-s1w)*j + s11*sz)*i;
  }
  case 5: { // below left
    uint32_t sH1 = b[s*H+1];
    uint32_t sh1 = b[s*h+1];
    i -= H;
    j += sz;
    return (sH1 + (sH1-sh1)*i + s11*sz)*j;
  }
  case 2: { // right
    uint32_t s1W = b[s + W];
    uint32_t s1w = b[s + w];
    uint32_t siW = b[s*i+W];
    uint32_t siw = b[s*i+w];
    uint32_t si1 = b[s*i+1];
    j -= W;
    return ( siW +
             (s11*sz + si1 + s1W)*sz +
             ((s1W - s1w)*sz + (siW - siw))*j );
  }
  case 4: { // below
    uint32_t sH1 = b[s*H+1];
    uint32_t sh1 = b[s*h+1];
    uint32_t sHj = b[s*H+j];
    uint32_t shj = b[s*h+j];
    uint32_t s1j = b[s + j];
    i -= H;
    return ( sHj +
             (s11*sz + s1j + sH1)*sz +
             ((sH1 - sh1)*sz + (sHj - shj))*i );
  }
  case 6: { // below right

    uint32_t s1W = b[s + W];
    uint32_t s1w = b[s + w];
    uint32_t sH1 = b[s*H+1];
    uint32_t sh1 = b[s*h+1];
    uint32_t sHW = b[s*H+W];
    uint32_t sHw = b[s*H+w];
    uint32_t shW = b[s*h+W];
    uint32_t shw = b[s*h+w];

    i -= H;
    j -= W;

    return ( sHW +
             (s11*sz + sH1 + s1W)*sz +
             ((sH1 - sh1)*sz + (sHW - shW)*(1+j) + (shw - sHw)*j)*i + 
             ((s1W - s1w)*sz + (sHW - sHw))*j ); // 14 add, 8 multiply

  }
  default:
    return 0;
  }

}

image_u32_t* integral_copy_border_replicate(const image_u32_t* iimg,
                                            int sz) {

  image_u32_t* copy = image_u32_create(iimg->width+2*sz, iimg->height+2*sz);

  uint32_t* dstrow = copy->buf;
  
  for (int y=0; y<copy->height; ++y) {
    for (int x=0; x<copy->width; ++x) {
      dstrow[x] = iimg_lookup_with_border(iimg, sz, y-sz, x-sz);
    }
    dstrow += copy->stride;
  }

  return copy;

}


image_u32_t* integrate(const image_u8_t* img) {

  image_u32_t tmp;
  tmp.width = img->width+1;
  tmp.height = img->height+1;
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

  for (int y=0; y<img->height; ++y) {

    *dst++ = 0;

    for (int x=0; x<img->width; ++x) {
      *dst = (*src++) + dst[o10] + dst[o01] - dst[o11];
      ++dst;
    }

    src += srcrem;
    dst += dstrem;

  }
  
  return iimg;


}

*/

void box_filter_border_replicate(const image_u8_t* src_img, 
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

  int s2 = sz*sz;
  int s22 = s2/2; // for rounding?
  
  image_u32_t* tmp_img = integrate_border_replicate(src_img, l);

  uint8_t* dst = dst_img->buf;

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
  
  int l = sz/2;
  sz = 2*l+1;

  int s2 = sz*sz;
  int s22 = s2/2; // for rounding?

  image_u32_t* tmp_img = integrate_border_replicate(src_img, l);

  uint8_t* dst = dst_img->buf;

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

}

typedef struct block_info {

  int bx; 
  int by;

  const uint8_t* src;
  
  int sstep;
  int sstride;
  
  uint32_t* dst;
  
  int nx;
  int ny;
  
  int is_central;
  int is_finished;
  
} block_info_t;

typedef struct integrate_info {

  pthread_mutex_t mutex;
  pthread_cond_t  cond;

  int dstride;
  
  int num_blocks_x;
  int num_blocks_y;
  int total_blocks;

  zarray_t* blocks;
  
  int* queue;

  int queue_start;
  int queue_end;
  int finished;
  
} integrate_info_t;

void integrate_task(void* p) {

  integrate_info_t* info = (integrate_info_t*)p;

  pthread_mutex_lock(&info->mutex);

  // invariant: we hold the lock at the top of the loop here
  while (!info->finished) { 

    if (info->queue_start == info->queue_end) {

      // if the queue is empty, wait for work to be placed into it (or
      // for all work to be finished)
      pthread_cond_wait(&info->cond, &info->mutex);
      // note we regain the lock after this returns
      
    } else {

      int idx = info->queue[info->queue_start];
      block_info_t* block;
      
      zarray_get_volatile(info->blocks, idx, &block);
        
      ++info->queue_start;

      pthread_mutex_unlock(&info->mutex);

      if (block->is_central) {
        integrate_block_center(block->src, block->sstride,
                               block->dst, info->dstride);
      } else {
        integrate_block(block->src, block->sstep, block->sstride,
                        block->dst, info->dstride,
                        block->nx, block->ny);
      }
      
      pthread_mutex_lock(&info->mutex);

      block->is_finished = 1;
      int do_broadcast = 0;

      if (idx != (info->total_blocks-1)) { // not all done

        // check above right 
        if (block->bx + 1 < info->num_blocks_x && block->by > 0) {
          int aridx = idx - info->num_blocks_x + 1;
          block_info_t* ar;
          zarray_get_volatile(info->blocks, aridx, &ar);
          if (ar->is_finished) {
            int ridx = idx + 1;
            info->queue[info->queue_end++] = ridx;
            do_broadcast = 1;
          }
        }

        // check below left
        if (block->by + 1 < info->num_blocks_y && block->bx > 0) {
          int blidx = idx + info->num_blocks_x - 1;
          block_info_t* bl;
          zarray_get_volatile(info->blocks, blidx, &bl);
          if (bl->is_finished) {
            int bidx = idx + info->num_blocks_x;
            info->queue[info->queue_end++] = bidx;
            do_broadcast = 1;
          }
        }

      } else { // all done

        do_broadcast = 1;
        info->finished = 1;
        
        
      }
      
    }
    
  }

  pthread_mutex_unlock(&info->mutex);

}
