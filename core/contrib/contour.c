#include "contour.h"
#include <stdio.h>


//#define DO_DEBUG

#ifdef DO_DEBUG
#define dprintf printf
#else
#define dprintf if (0) printf
#endif

/*
#ifdef NDEBUG
#undef NDEBUG
#endif
*/

#include <assert.h>

typedef int16_t ccount_t;

enum {
  CCOUNT_MAX = INT16_MAX
};


typedef struct cimage {
  
  const int width, height;
  const int stride;

  ccount_t* const buf;

} cimage_t;

cimage_t* cimage_create(int width, int height) {
  
  const int alignment = 64 / sizeof(ccount_t);

  int stride = width;

  if (stride % alignment) {
    stride += alignment - (stride%alignment);
  }

  ccount_t* buf = (ccount_t*)malloc(height*stride*sizeof(ccount_t));
  
  assert(buf);
  if (!buf) {
    fprintf(stderr, "out of memory!");
    return NULL;
  }

  cimage_t tmp = { width, height, stride, buf };

  cimage_t* rval = (cimage_t*)malloc(sizeof(cimage_t));

  assert(rval);
  if (!rval) {
    fprintf(stderr, "out of memory!");
    return NULL;
  }

  memcpy(rval, &tmp, sizeof(cimage_t));
  
  return rval;
  
}

void cimage_destroy(cimage_t* im) {

  if (!im) {
    return;
  }

  free(im->buf);
  free(im);
  
}

enum {
  NUM_NEIGHBORS = 8,
  NEIGHBOR_MASK = 0x7,
};

typedef struct conn_info {

  int lookup[3][3];
  int offs[8];
  int di[8];
  int dj[8];

} conn_info_t;

static const conn_info_t the_cinfo = {
  {
    {  3,  2,  1 },
    {  4, -1,  0 },
    {  5,  6,  7 }
  },
  {  0,  0,  0,  0,  0,  0,  0,  0 },
  {  0, -1, -1, -1,  0,  1,  1,  1 },
  {  1,  1,  0, -1, -1, -1,  0,  1 },
};

static inline int conn_lookup(const conn_info_t* c,
                              int isrc, int jsrc,
                              int idst, int jdst) {

  int di = idst-isrc;
  int dj = jdst-jsrc;

  assert(di >= -1 && di <= 1 && dj >= -1 && dj <= 1);

  int idx = c->lookup[di+1][dj+1];

  assert(idx >= 0 && idx < NUM_NEIGHBORS);
  assert(c->di[idx] == di && c->dj[idx] == dj);

  return idx;

}

static inline int conn_scan_cw(const conn_info_t* c,
                               const cimage_t* im,
                               uint32_t i,  uint32_t j,
                               uint32_t i2, uint32_t j2,
                               uint32_t* i1, uint32_t* j1) {
  
  int n = conn_lookup(c, i, j, i2, j2);

  assert(n >= 0 && n < NUM_NEIGHBORS);
  assert(i < (uint32_t)im->height && j < (uint32_t)im->width);

  int offs = i*im->stride + j;

  for (int k=0; k<NUM_NEIGHBORS; ++k) {
    
    ccount_t v = im->buf[offs + c->offs[n]];

    if (v) {
      *i1 = i + c->di[n];
      *j1 = j + c->dj[n];
      return 1;
    }
    
    n = (n + NEIGHBOR_MASK) & NEIGHBOR_MASK;
    
  }

  return 0;


}

static inline int conn_scan_ccw(const conn_info_t* c,
                                const cimage_t* im,
                                uint32_t i3, uint32_t j3,
                                uint32_t i2, uint32_t j2,
                                uint32_t* i4, uint32_t* j4,
                                int* right_is_zero) {
  
  assert(right_is_zero);
  *right_is_zero = 0;

  int n = conn_lookup(c, i3, j3, i2, j2);

  n = (n+1) & NEIGHBOR_MASK;

  assert(n >= 0 && n < NUM_NEIGHBORS);
  assert(i3 < (uint32_t)im->height && j3 < (uint32_t)im->width);

  int offs = i3*im->stride + j3;

  for (int k=0; k<NUM_NEIGHBORS; ++k) {
    
    ccount_t v = im->buf[offs + c->offs[n]];

    if (v) { 
      *i4 = i3 + c->di[n];
      *j4 = j3 + c->dj[n];
      return 1;
    } else if (n == 0) {
      *right_is_zero = 1;
    }
    
    n = (n + 1) & NEIGHBOR_MASK;
    
  }

  return 0;

}

static const int debug_print_grid = 1;

static void debug_print_hline(FILE* fp, int w) {
  fprintf(fp, debug_print_grid ? "+" : " ");
  for (int x=0; x<w; ++x) {
    fprintf(fp, debug_print_grid ? "---+" : "    ");
  }
  fprintf(fp, "\n");
}

void debug_print_image(FILE* fp, const cimage_t* im) {

  debug_print_hline(fp, im->width);
  
  const char* rowptr = (const char*)im->buf;

  for (int y=0; y<im->height; ++y) {

    fprintf(fp, debug_print_grid ? "|" : " ");

    for (int x=0; x<im->width; ++x) {

      int n = rowptr[x];
      char a = n < 0 ? '-' : ' ';
      char b = n ? abs(n) > 15 ? '?' : "0123456789ABCDEF"[abs(n)] : ' ';
      char c = debug_print_grid ? '|' : ' ';

      fprintf(fp, "%c%c %c", a, b, c);

    }

    fprintf(fp, "\n");

    debug_print_hline(fp, im->width);

    rowptr += im->stride;

  }

}

zarray_t* contour_detect(const image_u8_t* im8) {
  

  cimage_t* im = cimage_create(im8->width, im8->height);

  const uint8_t* src = im8->buf+im8->stride;
  ccount_t* dst = im->buf+im->stride;

  memset(im->buf, 0, sizeof(ccount_t)*im->width);
  memset(im->buf + im->stride, 0, sizeof(ccount_t)*im->width);
  memset(im->buf + (im->height-2)*im->stride, 0, sizeof(ccount_t)*im->width);
  memset(im->buf + (im->height-1)*im->stride, 0, sizeof(ccount_t)*im->width);

  for (uint32_t y=1; y<(uint32_t)(im->height-1); ++y) {
    for (uint32_t x=1; x<(uint32_t)(im->width-1); ++x) {
      dst[x] = src[x] ? 1 : 0;
    }
    dst[0] = dst[1] = 0;
    dst[im->width-2] = dst[im->width-1] = 0;
    src += im8->stride;
    dst += im->stride;
  }

  conn_info_t c = the_cinfo;
  
  for (int n=0; n<NUM_NEIGHBORS; ++n) {
    c.offs[n] = c.di[n]*im->stride + c.dj[n];
  }

#ifdef DO_DEBUG
  printf("at start of find_contours:\n");
  debug_print_image(stdout, im);
#endif

  zarray_t* contours = zarray_create(sizeof(contour_info_t));

  int nbd = 1;
  
  ccount_t* f_i = im->buf;

  // for each row
  for (uint32_t i=0; i<(uint32_t)im->height; ++i, f_i += im->stride) {

    // reset LNBD at start of row
    int lnbd = 1;

#ifdef DO_DEBUG
    printf("resetting lnbd=%d at start of row\n", lnbd);
#endif

    // for each column
    for (uint32_t j=0; j<(uint32_t)im->width; ++j) {

      // flag for border
      int is_border = 0;
      int is_outer = 0;

      // current border row/col
      uint32_t i2=i, j2;

      if (f_i[j] == 1 && f_i[j-1] == 0) {

        // 1a) if cur is 1, prev is 0 (outer border)
        j2 = j-1;
        is_border = 1;
        is_outer = 1;

      } else if (f_i[j] >= 1 && f_i[j+1] == 0) {

        // 1b) else if cur is >= 1, next is 0 (hole border)
        j2 = j+1;
        if (f_i[j] > 1) {
          lnbd = f_i[j];
#ifdef DO_DEBUG
          printf("reset lnbd=%d upon start of hb\n", lnbd);
#endif
        }
        is_border = 1;

      } 

      if (is_border) {

        if (nbd == CCOUNT_MAX) {
          fprintf(stderr, "warning: too many borders in contour_detect (max of %d!)\n", CCOUNT_MAX);
          contour_destroy(contours);
          cimage_destroy(im);
          return NULL;
        }
        
        ++nbd;

#ifdef DO_DEBUG

        printf("starting border following for border %d of type %s at %d, %d\n",
               nbd, f_i[j] ? "ob" : "hb", i, j);

        debug_print_image(stdout, im);

#endif

        contour_info_t cur_info;
        memset(&cur_info, 0, sizeof(cur_info));

        cur_info.is_outer = is_outer;

        // default parent is -1, which is frame (not added)
        cur_info.parent = -1;

        cur_info.points = zarray_create(sizeof(contour_point_t));

        // Implement table 1 from paper
        if (lnbd >= 2) {

          int lnbd_idx = lnbd-2;
          
          const contour_info_t* lnbd_info = NULL;

          assert(lnbd_idx >= 0);
          assert(lnbd_idx < zarray_size(contours));
          zarray_get_volatile(contours, lnbd_idx, &lnbd_info);

          // if both outer or both hole, take parent from lnbd
          if (cur_info.is_outer == lnbd_info->is_outer) {
            cur_info.parent = lnbd_info->parent;
          } else { // otherwise, parent IS lnbd
            cur_info.parent = lnbd_idx;
          }

        }

        //////////////////////////////////////////////////
        // Step 3

        //////////////////////////////////////////////////
        // 3.1)

#ifdef DO_DEBUG
        printf("at i=%d, j=%d, i2=%d, j2=%d\n", i, j, i2, j2);
#endif
        uint32_t i1, j1;
        int found_nonzero = conn_scan_cw(&c, im, i, j, i2, j2, &i1, &j1);

        if (!found_nonzero) {

          // end of 3.1
          f_i[j] = -nbd;
          
          contour_point_t point = { j, i };
          zarray_add(cur_info.points, &point);

        } else { 

#ifdef DO_DEBUG
          printf("found next clockwise nonzero neighbor at i1=%d, j1=%d\n", 
                 i1, j1);
#endif

          //////////////////////////////////////////////////
          // 3.2)
          i2 = i1; j2 = j1;

          uint32_t i3 = i;
          uint32_t j3 = j;

#ifdef DO_DEBUG
          printf("now i2=%d, j2=%d, i3=%d, j3=%d\n", i2, j2, i3, j3);
#endif

          // at start, i3=i, j3=j

          while (1) {

            contour_point_t point = { j3, i3 };

            zarray_add(cur_info.points, &point);

            //////////////////////////////////////////////////
            // 3.3)

            int right_is_zero = 0;
            uint32_t i4, j4;

            found_nonzero = conn_scan_ccw(&c, im, i3, j3, i2, j2, &i4, &j4, 
                                          &right_is_zero);

            assert(found_nonzero);

#ifdef DO_DEBUG
            printf("at current pixel i3=%d, j3=%d, next non-zero CCW is i4=%d, j4=%d\n", i3, j3, i4, j4);
#endif

            int offs = i3*im->stride + j3;

            //////////////////////////////////////////////////
            // 3.4
            if (right_is_zero) {
              im->buf[offs] = -nbd;
#ifdef DO_DEBUG
              printf("setting i3=%d, j3=%d to %d\n", i3, j3, im->buf[offs]);
              debug_print_image(stdout, im);
#endif
            } else if (!right_is_zero && im->buf[offs] == 1) {
              im->buf[offs] = nbd;
#ifdef DO_DEBUG
              printf("setting i3=%d, j3=%d to %d\n", i3, j3, im->buf[offs]);
              debug_print_image(stdout, im);
#endif
            } 

            //////////////////////////////////////////////////
            // 3.5

            if (i4 == i && j4 == j && i3 == i1 && j3 == j1) {
              // done when next i3, j3 would be i, j
#ifdef DO_DEBUG
              printf("all done with %d\n", nbd);
#endif
              break;
            } else {
              i2 = i3; j2 = j3;
              i3 = i4; j3 = j4;
              // loop back to 3.3
            }
              
          } // while following contour

        } // found_nonzero
        
        zarray_add(contours, &cur_info);

      } // is_border

      // 4)

      ccount_t afij = abs(f_i[j]);
      if (afij > 1) {
#ifdef DO_DEBUG
        if (lnbd != abs(f_i[j])) {
          printf("updating lnbd = %d\n", abs(f_i[j]));
        }
#endif
        lnbd = afij;
      }

    } // for each col

  } // for each row

#ifdef DO_DEBUG

  debug_print_image(stdout, im);

#endif

  cimage_destroy(im);
  
  return contours;

}

void contour_destroy(zarray_t* contours) {

  int nc = zarray_size(contours);

  for (int i=0; i<nc; ++i) {
    contour_info_t* ci;
    zarray_get_volatile(contours, i, &ci);
    zarray_destroy(ci->points);
  }
  
  zarray_destroy(contours);

}

static inline int compare_points(const void* va, const void* vb) {

  const contour_point_t* pa = (const contour_point_t*)va;
  const contour_point_t* pb = (const contour_point_t*)vb;

  if (pa->x < pb->x) {
    return -1;
  } else if (pa->x > pb->x) {
    return 1;
  } else if (pa->y < pb->y) {
    return -1;
  } else {
    return (pa->y > pb->y);
  }

}

static inline int turn(const contour_point_t* p,
                       const contour_point_t* q,
                       const contour_point_t* r) {

  return ( (q->x - p->x)*(r->y - p->y) - (r->x - p->x)*(q->y - p->y) );

}

size_t unique(void* base, size_t nel, size_t width, 
              int(*compar)(const void*, const void*)) {

  char* first = base;
  const char* last = base + nel * width;

  if (last == first) { 
    return 0;
  }

  char* result = first;
  size_t nout = 0;

  while ((first += width) != last) {

    if (compar(result, first)) {
      result += width;
      nout += 1;
      memmove(result, first, width);
    }

  }

  return nout + 1;

}


zarray_t* contour_convex_hull(const zarray_t* orig_points) {

  zarray_t* sorted = zarray_copy(orig_points);

  zarray_sort(sorted, compare_points);

  int num_unique = unique(sorted->data, 
                          sorted->size, 
                          sizeof(contour_point_t), 
                          compare_points);

  assert(num_unique >= 0 && num_unique <= zarray_size(sorted));
  zarray_truncate(sorted, num_unique);

  if (zarray_size(sorted) < 3) {
    // done!
    return sorted;
  }

  //////////////////////////////////////////////////
  // build top hull

  zarray_t* dst = zarray_create(sizeof(contour_point_t));

  for (int i=0; i<zarray_size(sorted); ++i) {

    const contour_point_t* pi;
    zarray_get_volatile(sorted, i, &pi);

    while (zarray_size(dst) >= 2) {

      const contour_point_t* d2;
      const contour_point_t* d1;
      
      zarray_get_volatile(dst, zarray_size(dst)-2, &d2);
      zarray_get_volatile(dst, zarray_size(dst)-1, &d1);

      if (turn(d2, d1, pi) > 0) {
        break;
      }

      zarray_truncate(dst, zarray_size(dst)-1);

    }

    zarray_add(dst, pi);

  }

  //////////////////////////////////////////////////
  // now do lower hull

  int tsize = zarray_size(dst);

  for (int i=zarray_size(sorted)-1; i>=0; --i) {

    const contour_point_t* pi;
    zarray_get_volatile(sorted, i, &pi);

    while (zarray_size(dst) >= tsize+2) {

      const contour_point_t* d2;
      const contour_point_t* d1;
      
      zarray_get_volatile(dst, zarray_size(dst)-2, &d2);
      zarray_get_volatile(dst, zarray_size(dst)-1, &d1);

      if (turn(d2, d1, pi) > 0) {
        break;
      }

      zarray_truncate(dst, zarray_size(dst)-1);

    }

    zarray_add(dst, pi);

  }

  //////////////////////////////////////////////////
  // remove 2 duplicate points

  zarray_truncate(dst, zarray_size(dst)-1);

  zarray_remove_index(dst, tsize, 0);

  zarray_destroy(sorted);

  return dst;

}


float contour_area_centroid(const zarray_t* points,
                            float centroid[2]) {

  int n = zarray_size(points);

  float area = 0;

  if (centroid) {
    centroid[0] = centroid[1] = 0.0;
  }

  for (int p=0; p<n; ++p) {

    const contour_point_t *pi, *pj;

    zarray_get_volatile(points, p, &pi);
    zarray_get_volatile(points, (p+1)%n, &pj);

    float xi = pi->x;
    float yi = pi->y;
    float xj = pj->x;
    float yj = pj->y;

    float dij = xi * yj - xj * yi;

    area += dij;
    
    if (centroid) {
      centroid[0] += (int)(xi + xj) * dij;
      centroid[1] += (int)(yi + yj) * dij;
    }
    
  }

  area *= 0.5;
  
  if (centroid) {
    float cscl = 6.0*area;
    centroid[0] /= cscl;
    centroid[1] /= cscl;
  }


  return area;

}

static inline void outer_boundary_points(int start, int end,
                                         int* nstart, int* ncount) {

  assert(start >= 0 && start < NUM_NEIGHBORS);
  assert(end >= 0 && end < NUM_NEIGHBORS);
  
  int diff_less_one = (end - start + NEIGHBOR_MASK + NUM_NEIGHBORS) & NEIGHBOR_MASK;
  
  if (start & 1) { // odd
    *ncount = (diff_less_one+1) / 2;
    *nstart = (start + 1) & NEIGHBOR_MASK;
  } else { // even
    *ncount = diff_less_one / 2;
    *nstart = (start + 2) & NEIGHBOR_MASK;
  }
  
  
}
  

zarray_t* contour_outer_boundary(const contour_info_t* cinfo,
                                 int start, int count) {

  int n = zarray_size(cinfo->points);
  if (n < 2) {
    fprintf(stderr, "won't get outer boundary of array of size less than 2!\n");
    return NULL;
  }

  const conn_info_t* c = &the_cinfo;

  const contour_point_t* p_cur;    
  const contour_point_t* p_prev;

  zarray_get_volatile(cinfo->points, (start+n-1)%n, &p_prev);
  zarray_get_volatile(cinfo->points, start, &p_cur);

  contour_point_t qprev = { -1, -1 };

  zarray_t* outer = zarray_create(sizeof(contour_point_t));

  for (int k=0; k<count; ++k) {
    
    const contour_point_t* p_next;
    zarray_get_volatile(cinfo->points, (start+k+1)%n, &p_next);

    // TODO: stuff
    int n_prev = conn_lookup(c,
                             p_cur->y, p_cur->x,
                             p_prev->y, p_prev->x);

    int n_next = conn_lookup(c,
                             p_cur->y, p_cur->x,
                             p_next->y, p_next->x);

    int nstart, ncount;
    outer_boundary_points(n_prev, n_next, &nstart, &ncount);

    /*
    {
      printf("prev is x=%d, y=%d\n", p_prev->x, p_prev->y);
      printf(" cur is x=%d, y=%d\n", p_cur->x, p_cur->y);
      printf("next is x=%d, y=%d\n", p_next->x, p_next->y);
      
      printf("(%d, %d) -> [ ", n_prev, n_next);
      int n = nstart;
      for (int i=0; i<ncount; ++i) {
        printf("%d ", n);
        n = (n + 2) & NEIGHBOR_MASK;
      }
      printf("]\n");
      printf("\n");
    }
    */

    int n=nstart;

    for (int i=0; i<ncount; ++i) {

      contour_point_t q;
      q.x = p_cur->x + c->dj[n];
      q.y = p_cur->y + c->di[n];


      if (q.x != qprev.x || q.y != qprev.y) {
        zarray_add(outer, &q);
        qprev = q;
      } else {
        int dx = (int)q.x - (int)qprev.x;
        int dy = (int)q.y - (int)qprev.y;
        if (!(dx >= -1 && dx <= 1 && dy >= -1 && dy <= 1)) {
          fprintf(stderr, "big skip!\n");
          exit(1);
        }
      }
      
      n = (n + 2) & NEIGHBOR_MASK;
      
    }

    
    p_prev = p_cur;
    p_cur = p_next;
    
  }

  if (zarray_size(outer)) {
    contour_point_t qbegin;
    zarray_get(outer, 0, &qbegin);
    if (qbegin.x == qprev.x && qbegin.y == qprev.y) {
      zarray_truncate(outer, zarray_size(outer) - 1);
    }
  }

  return outer;

}



void contour_test_outer() {


  /*
  Starting from 0:

  +---+---+---+---+---+---+---+---+---+
  |.o.|.o.|.o.|.o.|.o.|*o.|.*.|..*|567|
  |o**|o**|o**|o**|***|.**|.**|.**|4X0|
  |.o.|.o*|.*.|*..|...|...|...|...|321|
  +---+---+---+---+---+---+---+---+---+
   0,0 0,1 0,2 0,3 0,4 0,5 0,6 0,7
  
  (start, end) -> [ border pixels ]
  (0,0) -> [ 6, 4, 2 ]
  (0,1) -> [ 6, 4, 2 ]
  (0,2) -> [ 6, 4 ]
  (0,3) -> [ 6, 4 ]
  (0,4) -> [ 6 ]
  (0,5) -> [ 6 ]
  (0,6) -> [ ]
  (0,7) -> [ ]

  +---+---+---+---+---+---+---+---+
  |...|.o.|.o.|.o.|.o.|*o.|.*.|..*|
  |.**|o*o|o*o|o*o|**o|.*o|.*o|.*o|
  |..*|.o*|.**|*.*|..*|..*|..*|..*|
  +---+---+---+---+---+---+---+---+
   1,0 1,1 1,2 1,3 1,4 1,5 1,6 1,7 
   
   (1,0) -> [ ]
   (1,1) -> [ 0, 6, 4, 2 ]
   (1,2) -> [ 0, 6, 4 ]
   (1,3) -> [ 0, 6, 4 ]
   (1,4) -> [ 0, 6 ]
   (1,5) -> [ 0 ]
   (1,6) -> [ 0 ]

   +---+---+---+---+---+---+---+---+
   |...|...|.o.|.o.|.o.|*o.|.*.|..*|
   |.**|.*.|o*o|o*o|**o|.*o|.*o|.*o|
   |.*.|.**|.*.|**.|.*.|.*.|.*.|.*.|
   +---+---+---+---+---+---+---+---+
   
   (2,0) -> [ ]
   (2,1) -> [ ]
   (2,2) -> [ 0, 6, 4 ]
   (2,3) -> [ 0, 6, 4 ]
   (2,4) -> [ 0, 6 ]
   (2,5) -> [ 0, 6 ]
   (2,6) -> [ 0 ]
   (2,7) -> [ 0 ]
   
 */

  for (int start=0; start<NUM_NEIGHBORS; ++start) {
    for (int end=0; end<NUM_NEIGHBORS; ++end) {

      int nstart, ncount;
      outer_boundary_points(start, end, &nstart, &ncount);
      
      printf("(%d, %d) -> [ ", start, end);
      int n = nstart;
      for (int i=0; i<ncount; ++i) {
        printf("%d ", n);
        n = (n + 2) & NEIGHBOR_MASK;
      }
      printf("]\n");
      
    }

    printf("\n");
        
  }

}

typedef struct contour_node contour_node_t;

struct contour_node {
  contour_point_t point;
  size_t succ;
};

static const size_t npos = -1;

static inline int node_valid(zarray_t* nodes, size_t x) {
  return x < (size_t)nodes->size;
}

static inline contour_node_t* node_get(zarray_t* nodes, int idx) {
  assert( node_valid(nodes, idx) );
  return (contour_node_t*)nodes->data + idx;
}

static inline size_t node_create(zarray_t* nodes, uint32_t x, uint32_t y, size_t nidx) {

  size_t idx = nodes->size;
  zarray_ensure_capacity(nodes, idx+1);
  ++nodes->size;

  contour_node_t* n = node_get(nodes, idx);
  n->point.x = x;
  n->point.y = y;
  n->succ = nidx;
  dprintf("    new point at (%d, %d) with successor at pos (%d, %d)\n",
          (int)x, (int)y,
          node_valid(nodes, nidx) ? (int)node_get(nodes, nidx)->point.x : -1,
          node_valid(nodes, nidx) ? (int)node_get(nodes, nidx)->point.y : -1);

  return idx;
  
}

static inline void node_join(zarray_t* nodes, size_t p, size_t q) {
  assert( node_valid(nodes, q) );
  node_get(nodes, p)->succ = q;
  dprintf("    point at (%d, %d) now has successor at (%d, %d)\n",
         (int)node_get(nodes, p)->point.x, 
         (int)node_get(nodes, p)->point.y, 
         (int)node_get(nodes, q)->point.x, 
         (int)node_get(nodes, q)->point.y);
}


static inline void new_contour(zarray_t* nodes, size_t** pciter,
                               int y, int c0, int c1) {

  // todo: deal
  size_t d = node_create(nodes, c1, y+1, npos);
  size_t c = node_create(nodes, c1, y+0, d);
  size_t b = node_create(nodes, c0, y+0, c);
  size_t a = node_create(nodes, c0, y+1, b);
  node_join(nodes, d, a);

  size_t* citer = *pciter;
  citer[0] = a;
  citer[1] = d;
  *pciter += 2;

  dprintf("  new contour for interval [%d,%d]\n", c0, c1);
            
}

static inline int find_next_zero(const uint8_t* buf, int len) {
  return strnlen((const char*)buf, len);
}

#define find_next_nonzero find_next_nonzero_word

static inline int find_next_nonzero_simple(const uint8_t* buf, int len) {

    for (int i=0; i<len; ++i) {
      if (buf[i]) { return i; }
    }
    return len;

}

/*
static inline int nz64(uint64_t k) {
  return __builtin_ctzll(k)/8;
}
*/

static inline int nz64(uint64_t k) {

  int s = !(k & 0xFFFFFFFF) * 32;
  s |= !((k >> s) & 0xFFFF) * 16;
  s |= !((k >> s) & 0xFF) * 8;

  return s/8;  
  
}

/*

#if (defined(__clang__) || defined(__gcc__)) && defined(__LITTLE_ENDIAN__)
        int w = __builtin_ctzll(k) / 8; // for little-endian
#elif (defined(__clang__) || defined(__gcc__)) && defined(__BIG_ENDIAN__)
        int w = __builtin_clzll(k) / 8; // for big-endian
#else
        int w = ( buf[0] ? 0 :
                  buf[1] ? 1 :
                  buf[2] ? 2 :
                  buf[3] ? 3 :
                  buf[4] ? 4 :
                  buf[5] ? 5 :
                  buf[6] ? 6 : 7 );
#endif



        return offs + w;
*/


static inline int find_next_nonzero_word(const uint8_t* buf, int len) {

  int offs = 0;
    
  if (len >= 16) {
    
    {
      int r = (size_t)buf % 8;
      offs = (8 - r) % 8;
      for (int i=0; i<offs; ++i) {
        if (buf[i]) { return i; }
      }
      buf += offs;
      len -= offs;
    }

    assert((size_t)buf % 8 == 0);

    int n8 = len / 8;
    for (int j=0; j<n8; ++j) {
      uint64_t k;
      memcpy(&k, buf, 8);
      if (k) {
        return offs + nz64(k);
      }
      buf += 8;
      offs += 8;
    }

    len -= n8 * 8;

  }
  
  for (int i=0; i<len; ++i) {
    if (buf[i]) { return offs+i; }
  }
  
  return offs+len;
  
}

zarray_t* contour_line_sweep(const image_u8_t* im) {

  zarray_t* nodes = zarray_create(sizeof(contour_node_t));
  zarray_t* contours = zarray_create(sizeof(zarray_t*));

  size_t* prev_edges = malloc(sizeof(size_t)*(im->width+1));
  size_t prev_count = 0;

  size_t* cur_edges = malloc(sizeof(size_t)*(im->width+1));

  const uint8_t* srcrow = im->buf;
  
  for (uint32_t y=0; y<(uint32_t)im->height; ++y) {

    dprintf("processing y=%d\n", y);

    size_t* citer = cur_edges;

    size_t* piter = prev_edges;
    const size_t* pend = prev_edges + prev_count;

    const uint8_t* src = srcrow;
    uint32_t c1 = 0;
    int remaining = im->width;

    while (remaining) {

      int cnt = find_next_nonzero(src, remaining);
#if 0
      int cnt2 = find_next_nonzero_simple(src, remaining);
      if (cnt != cnt2) {
        fprintf(stderr,"error! cnt=%d, cnt2=%d\n", cnt, cnt2);
        exit(1);
      }
#endif
      
      if (cnt == remaining) {
        break;
      }

      src += cnt;
      uint32_t c0 = c1 + cnt;
      remaining -= cnt;

      cnt = find_next_zero(src, remaining);
      src += cnt;
      c1 = c0 + cnt;
      remaining -= cnt;

      dprintf("current interval is x=[%d,%d]\n", c0, c1);

      while (piter != pend && node_get(nodes, piter[1])->point.x <= c0) {
        piter += 2;
      }

      if (piter == pend) {

        dprintf("  starting new contour since piter == pend\n");
        new_contour(nodes, &citer, y, c0, c1);

      } else {

        assert(piter < pend);
            
        dprintf("  prev interval is x=[%d,%d]\n",
                (int)node_get(nodes, piter[0])->point.x,
                (int)node_get(nodes, piter[1])->point.x);
            
        assert(c0 < node_get(nodes, piter[1])->point.x);
            
        if (c1 <= node_get(nodes, piter[0])->point.x) {

          dprintf("  starting new contour since c1 <= p0\n");
          new_contour(nodes, &citer, y, c0, c1);
              
        } else {

          { // set up scope for pp0

            contour_node_t* pp0 = node_get(nodes, piter[0]);

            dprintf("  joining left...\n");
            // join left
            if (pp0->point.x == c0) {
              // just modify existing thing
              citer[0] = piter[0];
              pp0->point.y = y+1;
              dprintf("    modifying left hand of prev since x's equal\n");
            } else {
              size_t q = node_create(nodes, c0, y, piter[0]);
              citer[0] = node_create(nodes, c0, y+1, q);
            }

          } // no need to refer to pp0 after this point

          // join interior
          while (piter + 2 != pend &&
                 node_get(nodes, piter[2])->point.x < c1) {
            dprintf("  joining interior...\n");
            node_join(nodes, piter[1], piter[2]);
            piter += 2;
          }

          contour_node_t* pp1 = node_get(nodes, piter[1]);

          // join right
          dprintf("  joining right...\n");
          if (pp1->point.x == c1) {
                
            dprintf("    modifying right hand of prev since x's equal\n");
            citer[1] = piter[1];
            pp1->point.y = y+1;
            pp1->succ = citer[0];
                
          } else {

            citer[1] = node_create(nodes, c1, y+1, citer[0]);

            size_t q = node_create(nodes, c1, y,   citer[1]);

            // must re-get pointer because it may have been invalidated
            // due to creation immediately above this line
            pp1 = node_get(nodes, piter[1]);

            pp1->succ = q;

            if (c1 < pp1->point.x) {
                  
              dprintf("    updating prev edge because c1 < p1\n");
              piter[0] = q;
                  
            }
                
          }

          citer += 2;
              
        } // c1 < p0
      } // prev interval exists
      
    } // for each column

    prev_count = (citer-cur_edges);
    
    // swap prev and cur edges
    size_t* tmp = prev_edges;
    prev_edges = cur_edges;
    cur_edges = tmp;

    // move to next row
    srcrow += im->stride;
    
  }

  int ocount = 0;
  
  for (int i=0; i<nodes->size; ++i) {
    contour_node_t* n = node_get(nodes, i);
    if (node_valid(nodes, n->succ)) {
      zarray_t* contour = zarray_create(sizeof(contour_point_t));
      zarray_add(contours, &contour);
      while (node_valid(nodes, n->succ)) {
        zarray_add(contour, &n->point);
        contour_node_t* nn = node_get(nodes, n->succ);
        n->succ = npos;
        n = nn;
        ++ocount;
      }
    }
  }

  dprintf("ocount=%d, nodes->size=%d\n", ocount, nodes->size);

  if (ocount != nodes->size) {
    fprintf(stderr, "that was odd!\n");
    exit(1);
  }

  free(prev_edges);
  free(cur_edges);
  zarray_destroy(nodes);
  return contours;

}


void contour_line_sweep_destroy(zarray_t* contours) {

  for (int i=0; i<zarray_size(contours); ++i) {
    zarray_t* contour;
    zarray_get(contours, i, &contour);
    free(contour);          
  }
    
  zarray_destroy(contours);

}
