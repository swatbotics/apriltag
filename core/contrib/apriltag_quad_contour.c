#include <math.h>
#include <float.h>
#include <stdint.h>
#include <limits.h>
#include "apriltag_quad_contour.h"
#include "contour.h"
#include "box.h"
#include "lm.h"


/* 

  TODO:
   - parallelize finding contours (if possible)
   - parallelize processing contours 

  DONE: 
   - april-style debug visualizations
   - parallelize integral images
   - parallelize box threshold

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

    if (p->x && p->y && p->x + 1 < im->width && p->y + 1 < im->height) {

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

    lines[i].u[0] = dx / d;
    lines[i].u[1] = dy / d;

    lines[i].p[0] = q->p[i][0] + 0.5f * lines[i].u[1];
    lines[i].p[1] = q->p[i][1] - 0.5f * lines[i].u[0];

  }

  return 1;

}

/*
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))


static inline void bbox(int n, const float* p, 
                        int *x0, int* x1, int *y0, int* y1) {

  *x0 = INT_MAX;
  *y0 = INT_MAX;
  *x1 = 0;
  *y1 = 0;

  for (int i=0; i<n; ++i) {
    
    float xi = p[0];
    float yi = p[1];
    p += 2;

    int fxi = floorf(xi), cxi = ceilf(xi);
    int fyi = floorf(yi), cyi = ceilf(yi);

    *x0 = MIN(*x0, fxi);
    *x1 = MAX(*x1, cxi);

    *y0 = MIN(*y0, fyi);
    *y1 = MAX(*y1, cyi);
    
  }

  
}


static void tanh_residual(int m, int n,
                          const double* p,
                          double* x,
                          double* J,
                          void* userdata) {

  assert(m == 5);

  const double nx = p[0];
  const double ny = p[1];
  const double  d = p[2];
  const double  r = p[3];
  const double  o = p[4];
  
  const double* xyi = (const double*)userdata;

  for (int i=0; i<n; ++i) {
    double u = nx*xyi[0] + ny*xyi[1] + d;
    double t = tanh(u);
    double dt_du = 1-t*t;
    x[i] = r*t + o - xyi[2];
    if (J) {
      J[0] = r*dt_du*xyi[0];
      J[1] = r*dt_du*xyi[1];
      J[2] = r*dt_du;
      J[3] = t;
      J[4] = 1;
      J += 5;
    }
    xyi += 3;
  }

}


static inline int lines_refine(const image_u8_t* im,
                               struct quad* q,
                               g2d_line_t lines[4]) {


  int x0, y0, x1, y1;
  bbox(4, q->p[0], &x0, &x1, &y0, &y1);

  int m = 2;

  x0 -= m;
  y0 -= m;

  x1 += m+1;
  y1 += m+1;

  x0 = MAX(1, x0);
  x1 = MIN(im->width-1, x1);

  y0 = MAX(1, y0);
  y1 = MIN(im->height-1, y1);
  
  double llen[4] = { 0, 0, 0, 0 };
  int pcount[4] = { 0, 0, 0, 0 };
  zarray_t* lpts[4] = { 0, 0, 0, 0 };
  xyw_moments_t mo[4];

  //////////////////////////////////////////////////////////////////////
  // redo lines

  // line i goes from point (i-1) to point (i)
  for (int i=0; i<4; ++i) {

    memset(mo+i, 0, sizeof(xyw_moments_t));
    
    int j = (i+3) & 3;
    
    double gx = q->p[i][0] - q->p[j][0];
    double gy = q->p[i][1] - q->p[j][1];
    
    double l = sqrt(gx*gx + gy*gy);

    g2d_line_t* li = lines + i;

    li->p[0] = q->p[j][0];
    li->p[1] = q->p[j][1];

    li->u[0] = gx/l;
    li->u[1] = gy/l;

    llen[i] = l;
    
    lpts[i] = zarray_create(sizeof(double)*3);

  }

  uint8_t* srcrow = im->buf + y0*im->stride + x0;

  int up = -im->stride;
  int dn =  im->stride;
  int lt = -1;
  int rt = 1;

  for (int y=y0; y<y1; ++y) {

    uint8_t* src = srcrow;
    double fy = y+0.5;

    for (int x=x0; x<x1; ++x) {

      uint8_t sxy = *src;

      double gx = src[rt] - src[lt];
      double gy = src[dn] - src[up];
      double gmag = sqrt(gx*gx + gy*gy);

      ++src;
      
      double fx = x+0.5;

      for (int i=0; i<4; ++i) {

        const g2d_line_t* li = lines + i;

        double ux = li->u[0];
        double uy = li->u[1];

        double nx = uy;
        double ny = -ux;

        double dx = fx - li->p[0];
        double dy = fy - li->p[1];

        // get distance along
        double lx = ux*dx + uy*dy;
        double ly = nx*dx - ny*dy;

        const double dlx = 1.5;
        const double dly = 2.5;

        if (ly >= -dly && ly <= dly && lx >= dlx && lx <= llen[i] - dlx) {
          ++pcount[i];
          double xyi[3] = { fx, fy, sxy/255.0 };
          zarray_add(lpts[i], xyi);
          xyw_accum(mo+i, fx, fy, gmag);
        }
        
      }
 
    }
    
    srcrow += im->stride;
  }

  int ok = 1;
  int redo = 0;

  for (int i=0; i<4; ++i) {

    zarray_t* lpi = lpts[i];

    if (!pcount[i]) {
      ok = 0;
    }

    if (ok) {

      g2d_line_t* li = lines + i;
      
      double ux = li->u[0];
      double uy = li->u[1];
      
      double nx = uy;
      double ny = -ux;
      double d = -(nx * li->p[0] + ny*li->p[1]);

      double imin = DBL_MAX, imax = -DBL_MAX;

      for (int j=0; j<zarray_size(lpi); ++j) {
        const double* xyi;
        zarray_get_volatile(lpi, j, &xyi);
        imin = MIN(imin, xyi[2]);
        imax = MAX(imax, xyi[2]);
      }

      double r = 0.5*(imax-imin);
      double o = 0.5*(imax+imin);
      double scl = 16.0;
      
      double params[5] = { scl*nx, scl*ny, scl*d, r, o };

      //lm_check_residual(5, MIN(10, zarray_size(lpi)), params,
      //tanh_residual, lpi->data);

      lm_opts_t opts;

      lm_opts_defaults(&opts);
      opts.lfunc = LM_LOSS_HUBER;
      opts.lparam = 0.1;
      
      lm_info_t info;

      //printf("\n");

      //printf("initial params: %10.4f %10.4f %10.4f %10.4f %10.4f\n",
      //params[0], params[1], params[2], params[3], params[4]);

      int lm_result = lm_der(5, zarray_size(lpi), params, tanh_residual,
                             100, &opts, &info, lpi->data);

      //printf("final params:   %10.4f %10.4f %10.4f %10.4f %10.4f\n",
      //params[0], params[1], params[2], params[3], params[4]);

      //printf("initial loss: %f\n", info.initial_loss);
      //printf("final loss:   %f\n", info.final_loss);
      //printf("iterations:   %d\n", info.num_iterations);
      //printf("termination:  %s\n", lm_result_to_string(info.stop_reason));

      if (lm_result == 0) {

        nx = params[0];
        ny = params[1];
        d = params[2];

        double nmag = sqrt(nx*nx + ny*ny);

        nx /= nmag;
        ny /= nmag;
        d /= nmag;

        redo = 1;

        li->u[0] = ny;
        li->u[1] = -nx;
        li->p[0] = -d*nx;
        li->p[1] = -d*ny;

      }
      
    }
    
    zarray_destroy(lpi);
    
  }

  if (ok && redo) {

    for (int i=0; i<4; ++i) {
      int j = (i+1)&3; 
      double p[2];
      g2d_line_intersect_line(lines+i, lines+j, p);
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

  }

  
  return ok;

}

*/

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
    } else {
      line_init_from_xyw(&moments, lines+i);
    }
        
  }

  return 1;

}



zarray_t* quads_from_contours(const apriltag_detector_t* td,
                              const image_u8_t* im,
                              const zarray_t* contours) {

  zarray_t* quads = zarray_create(sizeof(struct quad));

  image_u32_t* debug_vis = NULL;
  
  if (td->debug) {
    debug_vis = im8_to_im32_dim(im, 0.5);
  }

  for (int c=0; c<zarray_size(contours); ++c) {

    contour_info_t* ci;
    zarray_get_volatile(contours, c, &ci);

    const int min_perimeter = 4*td->qcp.min_side_length;
    const float min_area = td->qcp.min_side_length * td->qcp.min_side_length;

    int n = zarray_size(ci->points);

    if (!ci->is_outer || n < min_perimeter) {
      continue;
    }

    int fail_reason = 0;

#define FAIL(r) do { fail_reason = (r); goto do_debug_vis; } while (0)

    float ctr[2];
    float area = fabs(contour_area_centroid(ci->points, ctr));

    // area check
    if (area < min_area) {
      FAIL(1);
    }
      
    int idx[4];
    float l, w;

    struct quad q = { .p={}, .H=0, .Hinv=0 };
    
    quad_from_points(ci->points, ctr, &q, idx, &l, &w);
      
    // diagonal aspect ratio check
    if (w < td->qcp.min_aspect*l) {
      FAIL(2);
    }

    float sides[4][2];
    float lmin, lmax;

    get_quad_sides(&q, sides, &lmin, &lmax);

    // side aspect ratio check
    if (lmin < td->qcp.min_aspect*lmax || lmin < td->qcp.min_side_length) {
      FAIL(3);
    }

    // max dist from quad check
    float dmax = get_max_quad_dist(ci->points, sides, &q);
    if (dmax > td->qcp.point_dist_diam_scl*l+td->qcp.point_dist_bias) {
      FAIL(4);
    }
      
    /*
      printf("got contour with n=%d with area=%f, diameter=%f, and dmax=%f\n", n, area, l, dmax);
      for (int i=0; i<4; ++i) {
      int j = (i+1)%4;
      printf("  contour[%d] = (%f, %f) with diff=%d\n",
      idx[i], q.p[i][0], q.p[i][1], (idx[j]-idx[i]+n)%n);
      }
      printf("\n");
    */

    int ok = 0;
    g2d_line_t lines[4];

    if (0) {
      ok = lines_from_corners_fast(&q, sides, lines);
    } else {
      ok = lines_from_corners_contour(td, im, ci, &q, idx, lines);
    }

    if (!ok) {
      FAIL(5);
    }

    for (int i=0; i<4; ++i) {
      lines[i].p[0] += 0.5;
      lines[i].p[1] += 0.5;
    }
    
    for (int i=0; i<4; ++i) {
      int j = (i+1)&3; 
      double p[2];
      g2d_line_intersect_line(lines+i, lines+j, p);
      q.p[i][0] = p[0];
      q.p[i][1] = p[1];
    }
      
    for (int i=0; ok && i<4; ++i) {
      int j = (i+1)&3;
      int k = (i+2)&3;
      float tval = turn(q.p[i], q.p[j], q.p[k]);
      if (tval < 0) {
        ok = 0;
        break;
      }
    }

    if (!ok) {
      FAIL(6);
    }


    get_quad_sides(&q, sides, &lmin, &lmax);
    if (lmin < td->qcp.min_aspect*lmax || lmin < td->qcp.min_side_length) {
      FAIL(7);
    }

    zarray_add(quads, &q);
    
    

  do_debug_vis:

    if (td->debug) {
      uint32_t colors[8] = {
        MAKE_RGB(255,   0, 255), // success = mid purple
        MAKE_RGB(127,   0,   0), // area = dark red
        MAKE_RGB(127,  63,   0), // diag. aspect = dark orange
        MAKE_RGB(127, 127,   0), // side aspect = dark yellow
        MAKE_RGB(  0, 127,   0), // too messy = dark green
        MAKE_RGB(  0, 127, 127), // not enough edge contrast = dark cyan
        MAKE_RGB(  0,   0, 127), // not CCW = dark blue
        MAKE_RGB(  0, 191,   0), // side 2 = mid green
      };
      draw_contour(debug_vis, ci->points, colors[fail_reason%8]);
    }

  }

  if (td->debug) {
    image_u32_write_pnm(debug_vis, "debug_quad_contour.pnm");
    image_u32_destroy(debug_vis);
  }

  return quads;

}


zarray_t* apriltag_quad_contour(apriltag_detector_t* td,
                                image_u8_t* im) {

  if (!td->quad_contours) {
    fprintf(stderr, "quad_contours is not set in tag detector!\n");
    assert(td->quad_contours);
    exit(1);
  }

  image_u8_t* thresh = box_threshold_mt(im, 255, 1,
                                        td->qcp.threshold_neighborhood_size,
                                        td->qcp.threshold_value,
                                        td->wp);

  if (td->debug) {
    image_u8_write_pnm(thresh, "debug_threshold.pnm");
  }

  timeprofile_stamp(td->tp, "threshold");

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

  zarray_t* quads = quads_from_contours(td, im, contours);
  timeprofile_stamp(td->tp, "quads from contours");

  contour_destroy(contours);
  image_u8_destroy(thresh);

  return quads;
  
}

