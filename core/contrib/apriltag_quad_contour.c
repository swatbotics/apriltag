#include <math.h>
#include <float.h>
#include "apriltag_quad_contour.h"
#include "contour.h"
#include "box.h"


/* TODO:

   - blank two border rows/cols in find contours to make sure no outer borders are on edge
   - parallelize ALL OF THE THINGS - contours, quads, etc
   - replace neighbor scan in outer border with LUT
   - instead of explicitly padding in box threshold, compute what padded table lookup ought to be
   - option for fast quad fitting using only corners (no XYW fit)
   - april-style debug visualizations

 */

#define MAKE_RGB(r, g, b) ((r)|((g)<<8)|((b)<<16))

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

    // get (0,0) as eigenvector :(
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

  float v[2] = { p0[1] - p2[1], 
                 p2[0] - p0[0] };

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

    // TODO: deal
    if (p->x == 0 || p->x == im->width-1 ||
        p->y == 0 || p->y == im->height-1) {
      continue;
    }

    int offs = p->x + im->stride*p->y;
    
    double gx = im->buf[offs+rt] - im->buf[offs+lt];
    double gy = im->buf[offs+dn] - im->buf[offs+up];

    double w = sqrt(gx*gx + gy*gy);

    isum += im->buf[offs]*w;
    wsum += w;

    xyw_accum(moments, p->x, p->y, w);
    
  }

  return isum / wsum;
  
}

static inline float turn(const float p[2],
                         const float q[2],
                         const float r[2]) {

  return ( (q[0] - p[0])*(r[1] - p[1]) - (r[0] - p[0])*(q[1] - p[1]) );
  
}

void apriltag_quad_contour_defaults(struct apriltag_quad_contour_params* qcp) {

  // 15 5 487
  // 23 1 490
  // 31 3 491
  qcp->threshold_neighborhood_size = 31;
  qcp->threshold_value = 3;
  qcp->min_side_length = 8;
  qcp->min_aspect = 0.1;
  qcp->point_dist_diam_scl = 0.1;
  qcp->point_dist_bias = 2.0;
  qcp->contour_margin = 5.0;
  
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


    int ok = 1;

    g2d_line_t lines[4];
    
    if (0) { 

      // quick and dirty: just dilate everything by half a pixel.
      // what could go wrong?
      for (int i=0; i<4; ++i) {

        double dx = sides[i][0];
        double dy = sides[i][1];

        double d = sqrt(dx*dx + dy*dy);

        double nx = -dy / d;
        double ny =  dx / d;

        lines[i].p[0] = q.p[i][0] + 0.5 * nx;
        lines[i].p[1] = q.p[i][1] + 0.5 * ny;

        lines[i].u[0] = dx;
        lines[i].u[1] = dy;
        
      }
    
    } else {

      // do line fit along border and outer border.
      for (int i=0; ok && i<4; ++i) {
        
        int j = (i+1)%4;
        int start = idx[i];
        int count = (idx[j]-idx[i]+n)%n;
        
        assert( count >= 6 );
        start = (start + 2) % n;
        count -= 4;

        xyw_moments_t moments;
        memset(&moments, 0, sizeof(moments));

        zarray_t* outer = contour_outer_boundary(ci, start, count);

        double mean_outer, mean_inner;
        mean_inner = sample_gradient_xyw(im, ci->points, start, count, &moments);
        mean_outer = sample_gradient_xyw(im, outer, 0, zarray_size(outer), &moments);

        zarray_destroy(outer);

        if ((mean_outer - mean_inner) < td->qcp.contour_margin) {
          ok = 0;
        } else {
          line_init_from_xyw(&moments, lines+i);
        }
        
      }

      if (!ok) {
        FAIL(5);
      }

    }
    
    for (int i=0; i<4; ++i) {
      int j = (i+1)&3; 
      double p[2];
      g2d_line_intersect_line(lines+i, lines+j, p);
      // note we are putting quads into order backwards here
      q.p[3-i][0] = p[0] + 0.5;
      q.p[3-i][1] = p[1] + 0.5;
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
    } else {
      zarray_add(quads, &q);
    }

  do_debug_vis:

    if (td->debug) {
      uint32_t colors[7] = {
        MAKE_RGB(255,   0, 255), // success = mid purple
        MAKE_RGB(127,   0,   0), // area = dark red
        MAKE_RGB(127,  63,   0), // diag. aspect = dark orange
        MAKE_RGB(127, 127,   0), // side aspect = dark yellow
        MAKE_RGB(  0, 127,   0), // too messy = dark green
        MAKE_RGB(  0, 127, 127), // not enough edge contrast = dark cyan
        MAKE_RGB(  0,   0, 127), // not CCW = dark blue
      };
      draw_contour(debug_vis, ci->points, colors[fail_reason%6]);
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

  image_u8_t* thresh = image_u8_create(im->width, im->height);

  box_threshold(im, thresh, 255, 1,
                td->qcp.threshold_neighborhood_size,
                td->qcp.threshold_value);

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

