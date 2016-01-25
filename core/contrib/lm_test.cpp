#include <stdio.h>
#include "lm.h"
#include <math.h>
#include <assert.h>
#include <stdlib.h>

inline void get_line_dist(const double p1p2[4], // (x1, y1, x2, y2)
                          const double p[], int npts, int pstride,
                          double* ldist, // (d1, ..., dn)
                          double* J1, int j1stride,
                          double* J2, int j2stride) {

  double x1 = p1p2[0];
  double y1 = p1p2[1];
  double x2 = p1p2[2];
  double y2 = p1p2[3];

  double dx = x2-x1;
  double dy = y2-y1;
  
  double l2 = dx*dx + dy*dy;
  double l = sqrt(l2);

  double nx =  dy/l;
  double ny = -dx/l;

  double l32 = pow(l2, 1.5);
  
  double dnx_dx1 =  dx*dy / l32;
  double dnx_dy1 = -dx*dx / l32;
  double dny_dx1 =  dy*dy / l32;
  double dny_dy1 = -dnx_dx1;

  for (int i=0; i<npts; ++i) {

    double x = p[0];
    double y = p[1];
    p += pstride;
    
    double px = x-x1;
    double py = y-y1;

    ldist[i] = px*nx + py*ny;


    if (J1) {
      J1[0] = (-nx + px*dnx_dx1 + py*dny_dx1);
      J1[1] = (-ny + px*dnx_dy1 + py*dny_dy1);
      J1 += j1stride;
    }

    if (J2) {
      J2[0] = -(px*dnx_dx1 + py*dny_dx1);
      J2[1] = -(px*dnx_dy1 + py*dny_dy1);
      J2 += j2stride;
    }

  }

}

void line_residual(int m, // # params
                   int n, // # residuals
                   const double* p1p2, // params
                   double* x, // residuals out
                   double* J, // jacobian out
                   void* userdata) {

  if (m != 4) {
    assert(m == 4);
    fprintf(stderr, "m != 4 in line_residual\n");
    exit(1);
  }

  const double* pts = (const double*)userdata;

  get_line_dist(p1p2, pts, n, 2, x,
                J ? J+0 : 0, 4,
                J ? J+2 : 0, 4);

}

int main(int argc, char** argv) {

  double p1p2[4] = { 0, 0, 11, 10 };

  enum {
    npts = 8
  };

  double p[npts*2] = {
    1.0, 1.1,
    2.1, 2.0,
    2.9, 3.1,
    4.1, 3.9,
    5.0, 6.0,
    6.0, 5.0,
    7.0, 7.0,
    1.0, 16.0
  };


  printf("Checking residual Jacobian...\n\n");
  lm_check_residual(4, npts, p1p2, line_residual, p);
  printf("\n");

  for (int l=0; l<LM_NUM_LOSS_FUNC; ++l){
    lm_loss_func_t lfunc = (lm_loss_func_t)l;
    printf("Checking loss gradient for %s (%d/%d)...\n\n",
           lm_loss_func_name(lfunc), l+1, LM_NUM_LOSS_FUNC);
    lm_check_loss(4, npts, p1p2, line_residual, p, lfunc, lm_lparam_default(lfunc));
    printf("\n");
  }


  lm_opts_t opts;
  lm_opts_defaults(&opts);

  opts.lfunc = LM_LOSS_HUBER;
  opts.lparam = 0.5;


  lm_check_loss(4, npts, p1p2, line_residual, p, opts.lfunc, opts.lparam);

  lm_der(4, npts, p1p2, line_residual, 100, &opts, NULL, p);

  printf("line is (%10.4f, %10.4f)-(%10.4f, %10.4f)\n",
         p1p2[0], p1p2[1], p1p2[2], p1p2[3]);

  double dx = p1p2[2]-p1p2[0];
  double dy = p1p2[3]-p1p2[1];
  double dmag = sqrt(dx*dx + dy*dy);
  
  dx /= dmag;
  dy /= dmag;

  printf("line has direction (%10.4f, %10.4f)\n", dx, dy);

  lm_check_loss(4, npts, p1p2, line_residual, p, opts.lfunc, opts.lparam);
  
  return 0;

}
