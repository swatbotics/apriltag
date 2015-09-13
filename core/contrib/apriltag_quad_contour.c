#include <math.h>
#include <float.h>
#include "apriltag_quad_contour.h"
#include "contour.h"

/* TODO:

   - blank two border rows/cols in find contours to make sure no outer borders are on edge
   - WLS to get border lines
   - intersect lines to get new endpoints

   Possible optimizations:

   1) parallelize ALL OF THE THINGS - contours, quads, etc
   2) replace neighbor scan in outer border with LUT
   4) instead of explicitly padding, compute what padded table lookup ought to be


   How to fit a line to a set of weighted points in a single pass?

          sum_i w_i * x_i
   m_x = -----------------
             sum_i w_i

 */


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

void fit_quad(const zarray_t* points,
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

// for easy2.png, orig apriltags identifies corners at (basically):
// 
//   p = { (36, 19), (59, 19), (59, 42), (36, 42) }
//
// which implies a CW ordering as seen by some viewer of the image:
// (TL, TR, BR, BL) but math-wise, it's CCW because underlying coord
// sys is left-handed)
//
// my program emits
//
//   p = { [36, 19], [59, 19], [59, 42], [36, 42] }
//
// which is great. only caveat is winding order for quad is 
// different than winding order for the contour. NBD.
//
zarray_t* quads_from_contours(const image_u8_t* im,
                              const zarray_t* contours) {

  zarray_t* quads = zarray_create(sizeof(struct quad));

  for (int c=0; c<zarray_size(contours); ++c) {

    contour_info_t* ci;
    zarray_get_volatile(contours, c, &ci);

    if (ci->is_outer && zarray_size(ci->points) >= 32) {

      float ctr[2];
      float area = contour_area_centroid(ci->points, ctr);
      area = fabs(area);

      // area check
      if (area < 64) { continue; }

      struct quad q;
      int idx[4];
      float l, w;
      
      fit_quad(ci->points, ctr, &q, idx, &l, &w);
      
      // diagonal aspect ratio check
      if (w < 0.3*l) { continue; }

      float sides[4][2];
      float lmin, lmax;

      get_quad_sides(&q, sides, &lmin, &lmax);

      // side aspect ratio check
      if (lmin < 0.3*lmax || lmin < 8) { continue; }

      // max dist from quad check
      float dmax = get_max_quad_dist(ci->points, sides, &q);
      if (dmax > 0.03*l+2) { continue; }
      

      int n = zarray_size(ci->points);

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

      for (int i=0; ok && i<4; ++i) {
        
        int j = (i+1)%4;
        int start = idx[i];
        int count = (idx[j]-idx[i]+n)%n;
        
        assert( count >= 8 );
        start = (start + 2) % n;
        count -= 4;

        xyw_moments_t moments;
        memset(&moments, 0, sizeof(moments));

        zarray_t* outer = contour_outer_boundary(ci, start, count);

        double mean_outer, mean_inner;
        mean_inner = sample_gradient_xyw(im, ci->points, start, count, &moments);
        mean_outer = sample_gradient_xyw(im, outer, 0, zarray_size(outer), &moments);

        zarray_destroy(outer);

        if ((mean_outer - mean_inner) < 10.0) {
          ok = 0;
        } else {
          line_init_from_xyw(&moments, lines+i);
        }
        
      }


      if (!ok) { continue; }

      for (int i=0; i<4; ++i) {
        int j = (i+1)&3; // note reverse order!
        double p[2];
        g2d_line_intersect_line(lines+i, lines+j, p);
        q.p[3-i][0] = p[0] + 0.5;
        q.p[3-i][1] = p[1] + 0.5;
      }

      zarray_add(quads, &q);
      
      
    }

  }

  return quads;

}
