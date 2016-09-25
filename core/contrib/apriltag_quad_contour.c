#include <math.h>
#include <float.h>
#include <stdint.h>
#include <limits.h>
#include "apriltag_quad_contour.h"
#include "contour.h"
#include "box.h"
#include "lm.h"
#include "math_util.h"

/* 

  TODO:

   - play with chunk size in box threshold?
   - parallelize finding contours 

  DONE: 

   - april-style debug visualizations
   - parallelize integral images
   - parallelize box threshold
   - parallelize processing contours (did not seem to help)

  NOPE:

   - replace neighbor scan in outer border with LUT - no because we
     dont spend any time doing that

   - option for fast quad fitting using only corners - tried, it
     sucked and shaved off an immeasurable amount

   - instead of explicitly padding in box threshold, compute what
     padded table lookup ought to be - nope, this was too branch-y

   - blank two border rows/cols in find contours to make sure no outer
     borders are on edge then we can take out if statement in
     sample_gradient_xyw - didn't save any time

   - refine pixels via XYW fit near lines after fast fit - no
     improvement

   - refine pixels via nonlinear least squares (fit border to tanh
     function) after fast fit - no improvement

 */

#define MAKE_RGB(r, g, b) ( ((b)<<16) | ((g)<<8) | ((r)<<0) )

uint32_t color_from_hue(double h) {

  uint32_t rgb[3];

  h -= floor(h);
  h *= 6.0;

  for (int i=0; i<3; ++i) {
    double ci = fmod(h + 2.0*i, 6.0);
    ci = fmin(ci, 4.0-ci);
    ci = fmax(0.0, fmin(ci, 1.0));
    rgb[i] = ci*255;
  }

  return MAKE_RGB(rgb[0], rgb[1], rgb[2]);

}

typedef struct xyw {
  float x, y, w;
} xyw_t;

typedef struct xyw_moments {
  double n;
  double mX;
  double mY;
  double mXX;
  double mYY;
  double mXY;
} xyw_moments_t;


void line_init_from_xyw(const xyw_moments_t* m,
                        g2d_line_t* line) {

  /*
  printf("n=%f, mX=%f, mY=%f, mXX=%f, mYY=%f, mXY=%f\n",
         m->n, m->mX, m->mY, m->mXX, m->mYY, m->mXY);
  */

  line->p[0] = m->mX / m->n;
  line->p[1] = m->mY / m->n;

  double Cxx = m->mXX/m->n - (m->mX/m->n)*(m->mX/m->n);
  double Cyy = m->mYY/m->n - (m->mY/m->n)*(m->mY/m->n);
  double Cxy = m->mXY/m->n - (m->mX/m->n)*(m->mY/m->n);

  double a = Cxx;
  double b = Cxy;
  double c = Cyy;

  if (b == 0) {

    if (a > c) {

      line->u[0] = 1;
      line->u[1] = 0;

    } else {

      line->u[0] = 0;
      line->u[1] = 1;

    }

  } else {

    double d = c*c - 2*a*c + 4*b*b + a*a;
  
    double v2x = sqrt(d)-c+a;
    double v2y = 2*b;
  
    double v2mag = sqrt(v2x*v2x + v2y*v2y);

    line->u[0] = v2x/v2mag;
    line->u[1] = v2y/v2mag;

  } 
  

}


float furthest_point(const zarray_t* points, 
                     const float q[2],
                     float p[2],
                     int* idx) {

  float max_r2 = -1;

  // get furthest from centroid
  for (int i=0; i<zarray_size(points); ++i) {

    const contour_point_t* pi;
    zarray_get_volatile(points, i, &pi);

    float xi = pi->x;
    float yi = pi->y;

    float dx = xi - q[0];
    float dy = yi - q[1];

    float r2 = dx*dx + dy*dy;

    if (r2 > max_r2) {
      max_r2 = r2;
      p[0] = xi;
      p[1] = yi;
      *idx = i;
    }

  }

  return max_r2;

}

float furthest_along(const zarray_t* points, 
                     const float q[2],
                     float p[2],
                     int* idx) {

  float max_d = -1;

  // get furthest from centroid
  for (int i=0; i<zarray_size(points); ++i) {
    const contour_point_t* pi;
    zarray_get_volatile(points, i, &pi);
    float xi = pi->x;
    float yi = pi->y;
    float di = q[0]*xi + q[1]*yi;
    if (i==0 || di > max_d) {
      max_d = di;
      p[0] = xi;
      p[1] = yi;
      *idx = i;
    }
  }

  return max_d;

}

void quad_from_points(const zarray_t* points,
                      const float ctr[2],
                      struct quad* q,
                      int idx[4],
                      float*l, float* w) {

  float* p0 = q->p[0];
  float* p1 = q->p[1];
  float* p2 = q->p[2];
  float* p3 = q->p[3];

  furthest_point(points, ctr, p0, idx+0);
  float l2 = furthest_point(points, p0, p2, idx+2);
  
  *l = sqrt(l2);

  float v[2] = { p2[1] - p0[1], 
                 p0[0] - p2[0] };

  v[0] /= *l;
  v[1] /= *l;

  float v1 = furthest_along(points, v, p1, idx+1);

  v[0] = -v[0];
  v[1] = -v[1];

  float v3 = -furthest_along(points, v, p3, idx+3);

  *w = fabs(v1-v3);

}

void get_quad_sides(const struct quad* q,
                    float sides[4][2],
                    float* lmin,
                    float* lmax) {

  *lmin = FLT_MAX;
  *lmax = 0;

  for (int i=0; i<4; ++i) {
    int j = (i+1)&3;
    sides[i][0] = q->p[j][0] - q->p[i][0];
    sides[i][1] = q->p[j][1] - q->p[i][1];
    float l = sqrt(sides[i][0]*sides[i][0] + sides[i][1]*sides[i][1]);
    *lmin = *lmin < l ? *lmin : l;
    *lmax = *lmax > l ? *lmax : l;
  }

  
}

float segment_dist2(const float a[2],
                    const float ba[2],
                    const float c[2]) {

  const float ca[2] = { c[0]-a[0], c[1]-a[1] };

  float ca_ba = ca[0]*ba[0] + ca[1]*ba[1];
  float ba_ba = ba[0]*ba[0] + ba[1]*ba[1];

  float u = ca_ba / ba_ba;
  u = u < 0.0 ? 0.0 : u > 1.0 ? 1.0 : u;

  float ux = a[0] + u*ba[0] - c[0];
  float uy = a[1] + u*ba[1] - c[1];

  /*
  printf("distance from point (%f, %f) to segment (%f, %f)-(%f, %f) is %f\n",
         p[0], p[1], p0[0], p0[1], p0[0]+dl[0], p0[1]+dl[1],
         sqrt(ux*ux + uy*uy));
  */

  return ux*ux + uy*uy;

}
                    

float get_max_quad_dist(const zarray_t* points,
                        const float sides[4][2],
                        const struct quad* q) {

  float d2max = 0.0;

  // get max over all points
  for (int i=0; i<zarray_size(points); ++i) {

    const contour_point_t* cp;
    zarray_get_volatile(points, i, &cp);
    
    const float p[2] = { cp->x, cp->y };

    float d2min = FLT_MAX;

    // get min over all sides
    for (int j=0; j<4; ++j) {
      float d2 = segment_dist2(q->p[j], sides[j], p);
      if (d2 < d2min) { d2min = d2; }
    }

    if (d2min > d2max) { d2max = d2min; }
    
  }

  return sqrt(d2max);
  
}

void xyw_accum(xyw_moments_t* m,
               double x, double y, double w) {

  //printf("%f %f %f\n", x, y, w);
  
  m->n += w;

  m->mX += x*w;
  m->mY += y*w;
  
  m->mXX += x*x*w;
  m->mYY += y*y*w;
  m->mXY += x*y*w;
  
}


double sample_gradient_xyw(const image_u8_t* im, 
                           const zarray_t* points,
                           int start,
                           int count,
                           xyw_moments_t* moments) {

  int n = zarray_size(points);

  const int up = -im->stride;
  const int dn =  im->stride;
  const int lt = -1;
  const int rt =  1;

  double isum = 0.0;
  double wsum = 0.0;

  for (int k=0; k<count; ++k) {

    int i = (start+k)%n;

    const contour_point_t* p;
    zarray_get_volatile(points, i, &p);

    if (p->x && p->y && p->x + 1 < (uint32_t)im->width && p->y + 1 < (uint32_t)im->height) {

      int offs = p->x + im->stride*p->y;
    
      double gx = im->buf[offs+rt] - im->buf[offs+lt];
      double gy = im->buf[offs+dn] - im->buf[offs+up];

      double w = sqrt(gx*gx + gy*gy);

      isum += im->buf[offs]*w;
      wsum += w;

      xyw_accum(moments, p->x, p->y, w);

    }
    
  }

  return isum / wsum;
  
}

static inline float turn(const float p[2],
                         const float q[2],
                         const float r[2]) {

  return ( (q[0] - p[0])*(r[1] - p[1]) - (r[0] - p[0])*(q[1] - p[1]) );
  
}

void apriltag_quad_contour_defaults(struct apriltag_quad_contour_params* qcp) {

  qcp->threshold_neighborhood_size = 15;
  qcp->threshold_value = 5;
  qcp->min_side_length = 8;
  qcp->min_aspect = 0.1;
  qcp->point_dist_diam_scl = 0.1;
  qcp->point_dist_bias = 2.0;
  qcp->contour_margin = 5.0;
  qcp->corner_skip_scl = 0.0625;
  qcp->corner_skip_bias = 0.5;
  
}

void draw_contour(image_u32_t* display,
                  const zarray_t* points,
                  uint32_t color) {
  
  for (int i=0; i<zarray_size(points); ++i) {
    const contour_point_t* p;
    zarray_get_volatile(points, i, &p);
    display->buf[p->x + display->stride*p->y] = color;
  }
  
}

image_u32_t* im8_to_im32_dim(const image_u8_t* im,
                             double scl) {

  scl = scl < 0 ? 0 : scl > 1 ? 1 : scl;

  uint32_t frac = 256*scl;

  image_u32_t* dst = image_u32_create(im->width, im->height);

  const uint8_t* srcrow = im->buf;
  uint32_t* dstrow = dst->buf;

  for (int y=0; y<im->height; ++y) {
    for (int x=0; x<im->width; ++x) {
      uint32_t ival = ((uint32_t)srcrow[x] * frac) >> 8;
      dstrow[x] = (ival << 24) | (ival << 16) | (ival << 8) | (ival);
    }
    srcrow += im->stride;
    dstrow += dst->stride;
  }

  return dst;

}

static inline int lines_from_corners_fast(const struct quad* q,
                                          const float sides[4][2],
                                          g2d_line_t lines[4]) {

  for (int i=0; i<4; ++i) {
    
    double dx = sides[i][0];
    double dy = sides[i][1];
    
    double d = sqrt(dx*dx + dy*dy);
    if (!d) { // prevent divide by 0
      return 0;
    }

    lines[i].u[0] = dx / d;
    lines[i].u[1] = dy / d;

    lines[i].p[0] = q->p[i][0] + 0.5f * lines[i].u[1];
    lines[i].p[1] = q->p[i][1] - 0.5f * lines[i].u[0];

  }

  return 1;

}

static inline int lines_from_corners_contour(const apriltag_detector_t* td,
                                             const image_u8_t* im,
                                             const contour_info_t* ci,
                                             const struct quad* q,
                                             const int idx[4],
                                             g2d_line_t lines[4]) {

  int n = zarray_size(ci->points);

  // do line fit along border and outer border.
  for (int i=0; i<4; ++i) {
        
    int j = (i+3)&3;
    int start = idx[i];
    int count = (idx[j]-idx[i]+n+1)%n;

    if (count < 8) { // not sure why below assert was here, added safety code?
      fprintf(stderr, "%s:%d: warning: count < 8 :(\n",
              __FILE__, __LINE__);
      return 0;
    }
        
    assert( count >= 8 );

    int corner_skip = count * td->qcp.corner_skip_scl + td->qcp.corner_skip_bias;
    corner_skip = corner_skip > count/4 ? count/4 : corner_skip;
        
    start = (start + corner_skip) % n;
    count -= 2*corner_skip;

    xyw_moments_t moments;
    memset(&moments, 0, sizeof(moments));

    zarray_t* outer = contour_outer_boundary(ci, start, count);

    double mean_outer, mean_inner;
    mean_inner = sample_gradient_xyw(im, ci->points, start, count, &moments);
    mean_outer = sample_gradient_xyw(im, outer, 0, zarray_size(outer), &moments);

    zarray_destroy(outer);

    if ((mean_outer - mean_inner) < td->qcp.contour_margin) {
      return 0;
    } else if (!moments.n) {
      return 0;
    } else {
      line_init_from_xyw(&moments, lines+i);
    }
        
  }

  return 1;

}

/* The workhorse of the quad detection: takes an individual contour and
   tries to extract a quad from it, with various types of rejection. */
static inline int quad_from_contour(const apriltag_detector_t* td,
                                    const image_u8_t* im,
                                    const contour_info_t* ci,
                                    struct quad* q) {


  /* Look at perimiter and area */
  const int min_perimeter = 4*td->qcp.min_side_length;
  const float min_area = td->qcp.min_side_length * td->qcp.min_side_length;

  /* Eliminate really tiny contours: if each side of the contour is 1
     pixel, we can lower-bound perimiter */
  int n = zarray_size(ci->points);

  if (!ci->is_outer || n < min_perimeter) {
    return -1;
  }

  /* Compute area and centroid. */
  float ctr[2];
  float area = fabs(contour_area_centroid(ci->points, ctr));

  // area check
  if (area < min_area) {
    return 1;
  }
      
  int idx[4];
  float l, w;

  q->H = 0;
  q->Hinv = 0;
    
  quad_from_points(ci->points, ctr, q, idx, &l, &w);
      
  // diagonal aspect ratio check
  if (w < td->qcp.min_aspect*l) {
    return 2;
  }

  float sides[4][2];
  float lmin, lmax;

  get_quad_sides(q, sides, &lmin, &lmax);

  // side aspect ratio check
  if (lmin < td->qcp.min_aspect*lmax || lmin < td->qcp.min_side_length) {
    return 2;
  }

  // max dist from quad check
  float dmax = get_max_quad_dist(ci->points, sides, q);
  if (dmax > td->qcp.point_dist_diam_scl*l+td->qcp.point_dist_bias) {
    return 4;
  }

  int ok = 0;
  g2d_line_t lines[4];

  if (0) {
    ok = lines_from_corners_fast(q, sides, lines);
  } else {
    ok = lines_from_corners_contour(td, im, ci, q, idx, lines);
  }

  if (!ok) {
    return 5;
  }

  for (int i=0; i<4; ++i) {
    lines[i].p[0] += 0.5;
    lines[i].p[1] += 0.5;
  }
    
  for (int i=0; i<4; ++i) {
    int j = (i+1)&3; 
    double p[2];
    int result = g2d_line_intersect_line(lines+i, lines+j, p);
    if (!result) { return 5; }
    q->p[i][0] = p[0];
    q->p[i][1] = p[1];
  }
      
  for (int i=0; ok && i<4; ++i) {
    int j = (i+1)&3;
    int k = (i+2)&3;
    float tval = turn(q->p[i], q->p[j], q->p[k]);
    if (tval < 0) {
      ok = 0;
      break;
    }
  }

  if (!ok) {
    return 6;
  }


  get_quad_sides(q, sides, &lmin, &lmax);
  if (lmin < td->qcp.min_aspect*lmax || lmin < td->qcp.min_side_length) {
    return 7;
  }

  return 0;
    

}

typedef struct qfc_info {
  const apriltag_detector_t* td;
  const image_u8_t* im;
  const contour_info_t* contours;
  struct quad* quads;
  int* results;
  int count;
} qfc_info_t;

/* Simple wrapper on quad_from_contour */
static void qfc_task(void* p) {

  qfc_info_t* qfc = (qfc_info_t*)p;

  for (int i=0; i<qfc->count; ++i) {
    qfc->results[i] = quad_from_contour(qfc->td, qfc->im,
                                        qfc->contours + i,
                                        qfc->quads + i);
  }
  
}

zarray_t* quads_from_contours(const apriltag_detector_t* td,
                              const image_u8_t* im,
                              const zarray_t* contours) {

  zarray_t* quads = zarray_create(sizeof(struct quad));

  image_u32_t* debug_vis = NULL;
  
  if (td->debug) {
    debug_vis = im8_to_im32_dim(im, 0.5);
  }

  int nc = zarray_size(contours);

  /* Step 3: divide list of contours into a worker pool, and tell
     workers to run the qfc_task, which is just a simple wrapper on
     quad_from_contour. */

  struct quad wquads[nc];
  int results[nc];
  const contour_info_t* ctrs = (const contour_info_t*)contours->data;

#if 1

  int chunksize = 1 + nc / (APRILTAG_TASKS_PER_THREAD_TARGET * td->nthreads);
  //int chunksize = 1 + nc / td->nthreads;

  qfc_info_t qfcs[nc / chunksize + 1];

  int ntasks = 0;

  for (int i=0; i<nc; i+=chunksize) {
    qfcs[ntasks].td = td;
    qfcs[ntasks].im = im;
    qfcs[ntasks].contours = ctrs + i;
    qfcs[ntasks].quads = wquads + i;
    qfcs[ntasks].results = results + i;
    qfcs[ntasks].count = imin(nc, i+chunksize) - i;
    workerpool_add_task(td->wp, qfc_task, qfcs+ntasks);
    ++ntasks;
  }

  workerpool_run(td->wp);

#else

  qfc_info_t p;
  p.td = td;
  p.im = im;
  p.contours = ctrs;
  p.quads = wquads;
  p.results = results;
  p.count = nc;
  qfc_task(&p);

#endif
  
  for (int c=0; c<nc; ++c) {
    
    if (results[c] == 0) {
      zarray_add(quads, wquads+c);
    }

    if (td->debug && results[c] >= 0) {
      uint32_t colors[8] = {
        MAKE_RGB(255,   0, 255), // success = mid purple
        MAKE_RGB(127,   0,   0), // min. area = dark red
        MAKE_RGB(127,  63,   0), // diag. aspect = dark orange
        MAKE_RGB(127, 127,   0), // side aspect = dark yellow
        MAKE_RGB(  0, 127,   0), // too messy = dark green
        MAKE_RGB(  0, 127, 127), // not enough edge contrast = dark cyan
        MAKE_RGB(  0,   0, 127), // not CCW = dark blue
        MAKE_RGB(  0, 191,   0), // side 2 = mid green
      };
      draw_contour(debug_vis, ctrs[c].points, colors[results[c] % 8]);
    }
    
  }

  if (td->debug) {
    image_u32_write_pnm(debug_vis, "debug_quad_contours.pnm");
  }

  return quads;

}


/* Main function added by Matt. */
zarray_t* apriltag_quad_contour(apriltag_detector_t* td,
                                image_u8_t* im) {

  if (!td->quad_contours) {
    fprintf(stderr, "quad_contours is not set in tag detector!\n");
    assert(td->quad_contours);
    exit(1);
  }

  /* Step 1: box blur & threshold (adaptive threshold) */
  image_u8_t* thresh = box_threshold_mt(im, 255, 1,
                                        td->qcp.threshold_neighborhood_size,
                                        td->qcp.threshold_value,
                                        td->wp);

  if (td->debug) {
    image_u8_write_pnm(thresh, "debug_threshold.pnm");
  }

  timeprofile_stamp(td->tp, "threshold");

  /* Step 2: contour detection */
  zarray_t* contours = contour_detect(thresh); 
  timeprofile_stamp(td->tp, "contour");

  if (td->debug) {
    image_u32_t* display = im8_to_im32_dim(im, 0.5);
    for (int c=0; c<zarray_size(contours); ++c) {
      const contour_info_t* ci;
      zarray_get_volatile(contours, c, &ci);
      uint32_t color = color_from_hue(c*0.5773502691896257);
      if (ci->is_outer) {
        draw_contour(display, ci->points, color);
      }
    }
    image_u32_write_pnm(display, "debug_contours.pnm");
    image_u32_destroy(display);
  }

  /* Steps 3-N: extract quads from contours (see above). */
  zarray_t* quads = quads_from_contours(td, im, contours);
  timeprofile_stamp(td->tp, "quads from contours");

  contour_destroy(contours);
  image_u8_destroy(thresh);

  return quads;
  
}

