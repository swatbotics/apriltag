#include "lm.h"
#include "matd.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <float.h>

// see http://research.microsoft.com/en-us/um/people/zhang/INRIA/Publis/Tutorial-Estim/node24.html

static inline double lm_loss_tukey(double x, double c, double* w) {

  double c2 = c*c;

  if (fabs(x) <= c) {
    double xc = x/c;
    double xc2 = xc*xc;
    double a = 1-xc2;
    double a2 = a*a;
    double a3 = a*a2;
    *w = a2;
    return (c2/6.0)*(1.0 - a3);
  } else {
    *w = 0.0;
    return (c2/6.0);
  }
  
}

static inline double lm_loss_l1(double x, double* w) {
  double ax = fabs(x);
  *w = 1/ax;
  return ax;
}

static inline double lm_loss_l1_minus_l2(double x, double* w) {
  double x22 = 0.5*x*x;
  double s = sqrt(1.0 + x22);
  *w = 1/s;
  return 2.0*(s-1.0);
}

static inline double lm_loss_least_power(double x, double v, double* w) {
  double ax = fabs(x);
  *w = pow(ax, v-2);
  return pow(ax, v)/v;  
}

static inline double lm_loss_huber(double x, double k, double* w) {
  double ax = fabs(x);
  if (ax <= k) {
    *w = 1.0;
    return 0.5*x*x;
  } else {
    *w = k/ax;
    return k*(ax-0.5*k);
  }
}

static inline double lm_loss_fair(double x, double c, double* w) {
  double ax = fabs(x);
  double c2 = c*c;
  double axc = ax/c;
  *w = 1.0 / (1.0 + axc);
  return c2*(axc - log(1.0 + axc));
}


double lm_lparam_default(lm_loss_func_t lfunc) {
  switch (lfunc) {
  case LM_LOSS_TUKEY:
    return 4.6851;
  case LM_LOSS_LEAST_POWER:
    return 1.2;
  case LM_LOSS_FAIR:
    return 1.3998;
  case LM_LOSS_HUBER:
    return 1.345;
  default:
    return 0.0;
  }
}

const char* lm_loss_func_name(lm_loss_func_t lfunc) {
  switch (lfunc) {
  case LM_LOSS_TUKEY:
    return "Tukey";
  case LM_LOSS_HUBER:
    return "Huber";
  case LM_LOSS_FAIR:
    return "fair";
  case LM_LOSS_LEAST_POWER:
    return "least power";
  case LM_LOSS_L1_MINUS_L2:
    return "L1-L2";
  case LM_LOSS_L1:
    return "L1";
    break;
  case LM_LOSS_L2:
  default:
    return "L2";
  }
}

static inline double lm_loss(lm_loss_func_t lfunc,
                             double x, double lparam, double* w) {
 
  switch (lfunc) {
  case LM_LOSS_TUKEY:
    return lm_loss_tukey(x, lparam, w);
  case LM_LOSS_HUBER:
    return lm_loss_huber(x, lparam, w);
  case LM_LOSS_FAIR:
    return lm_loss_fair(x, lparam, w);
  case LM_LOSS_LEAST_POWER:
    return lm_loss_least_power(x, lparam, w);
  case LM_LOSS_L1_MINUS_L2:
    return lm_loss_l1_minus_l2(x, w);
  case LM_LOSS_L1:
    return lm_loss_l1(x, w);
    break;
  case LM_LOSS_L2:
  default:
    *w = 1.0;
    return 0.5*x*x;
  }

}


void lm_check_residual(int m, // size of p
                       int n, // size of x
                       double* p, // params
                       lm_res_func_t rfunc,
                       void* userdata) {

  matd_t* x  = matd_create(n, 1);
  matd_t* J  = matd_create(n, m);
  matd_t* Jn = matd_create(n, m);

  rfunc(m, n, p, x->data, J->data, userdata);

  double h = 1e-5;

  for (int j=0; j<m; ++j) {

    double xp[n], xn[n];

    double pj = p[j];
    
    p[j] = pj + h;

    rfunc(m, n, p, xp, NULL, userdata);

    p[j] = pj - h;

    rfunc(m, n, p, xn, NULL, userdata);
    
    p[j] = pj;

    for (int i=0; i<n; ++i) {
      Jn->data[m*i + j] = (xp[i]-xn[i])/(2.0*h);
    }
    
  }

  printf("Analytic Jacobian transpose:\n");
  matd_print_transpose(J, "%10.4f ");
  printf("\n");

  printf("Numerical Jacobian transpose:\n");
  matd_print_transpose(Jn, "%10.4f ");
  printf("\n");

  printf("max error: %10g\n", matd_err_inf(J, Jn));
  
  matd_destroy(x);
  matd_destroy(J);
  matd_destroy(Jn);
  
}

static inline double lm_debug_res_to_loss(int m,
                                          int n,
                                          double* p,
                                          lm_res_func_t rfunc,
                                          void* userdata,
                                          lm_loss_func_t lfunc,
                                          double lparam,
                                          double* g) {

  double loss = 0;
  double J[m*n];
  double x[n];

  rfunc(m, n, p, x, g ? J : 0, userdata);

  if (g) {
    memset(g, 0, sizeof(double)*m);
  }

  for (int i=0; i<n; ++i) {

    double li, wi;
    li = lm_loss(lfunc, x[i], lparam, &wi);

    if (li < 0.0) {
      assert(li >= 0.0);
      fprintf(stderr, "got negative loss in lm_debug_res_to_loss\n");
    }

    loss += li;

    if (g) {
      for (int j=0; j<m; ++j) {
        g[j] += x[i] * J[m*i + j] * wi;
      }
    }
    
  }

  return loss;

}

void lm_check_loss(int m, // size of p
                   int n, // size of x
                   double* p, // params
                   lm_res_func_t rfunc,
                   void* userdata,
                   lm_loss_func_t lfunc,
                   double lparam) {

  matd_t* g = matd_create(m, 1);
  matd_t* gn = matd_create(m, 1);

  lm_debug_res_to_loss(m, n, p, rfunc, userdata,
                       lfunc, lparam, g->data);

  double h = 1e-4;

  for (int j=0; j<m; ++j) {


    double pj = p[j];
    
    p[j] = pj + h;

    double lp = lm_debug_res_to_loss(m, n, p, rfunc, userdata,
                                     lfunc, lparam, 0);


    p[j] = pj - h;

    double lm = lm_debug_res_to_loss(m, n, p, rfunc, userdata,
                                     lfunc, lparam, 0);
    
    p[j] = pj;

    gn->data[j] = (lp - lm) / (2.0*h);
    
  }

  printf("Analytic gradient transpose:\n");
  matd_print_transpose(g, "%10.4f ");
  printf("\n");

  printf("Numerical gradient transpose:\n");
  matd_print_transpose(gn, "%10.4f ");
  printf("\n");

  printf("max error: %10g\n", matd_err_inf(g, gn));
  
  matd_destroy(g);
  matd_destroy(gn);

}

void lm_opts_defaults(lm_opts_t* opts) {
  opts->JTJ_sing_tol = 1e-6;
  opts->JT_err_infnorm_tol = 1e-12;
  opts->delta_p_rel_infnorm_tol = 1e-7;
  opts->loss_tol = DBL_MIN;
  opts->lfunc = LM_LOSS_L2;
  opts->lparam = 0.0;
}

const char* lm_result_to_string(int result) {
  switch (result) {
  case LM_SUCCESS:
    return "success";
  case LM_MAX_ITER:
    return "maximum iterations exceeded.";
  case LM_SINGULAR_MATRIX:
    return "singular matrix";
  case LM_STOP_JT_ERR:
    return "loss gradient close to zero";
  case LM_STOP_DELTA_P:
    return "parameter step close to zero";
  case LM_STOP_LOSS:
    return "loss close to zero";
  default:
    return "???";
  }
}

static inline
void transpose_product_inplace(const matd_t* A, 
                               const matd_t* W,
                               const matd_t* X,
                               matd_t* B) {

  // compute A^T * X = B
  //
  // A^T is m-by-n (meaning A is n-by-m)
  // X is n-by-p
  // B is m-by-p
  //
  // A has m columns of length n
  // X has p columns of length n

  int n = A->nrows;
  int m = A->ncols;
  int p = X->ncols;

  if (W) { assert(W->nrows == n && W->ncols == 1); }
  assert(X->nrows == n);
  assert(B->nrows == m && B->ncols == p);

  // for each row of output
  for (int i=0; i<m; ++i) { 

    // for each column of output
    for (int j=0; j<p; ++j) {

      // B at (i, j) is the inner product of the i'th column of A with
      // the j'th column of X
      double Bij = 0.0;

      if (W) { 
        for (int k=0; k<n; ++k) {
          Bij += MATD_EL(A,k,i) * MATD_EL(X,k,j) * W->data[k];
        }
      } else {
        for (int k=0; k<n; ++k) {
          Bij += MATD_EL(A,k,i) * MATD_EL(X,k,j);
        }
      }
        
      MATD_EL(B,i,j) = Bij;

    }
    
  }


}

double inf_norm(const matd_t* A) {

  int sz = A->nrows*A->ncols;
  double n = 0.0;
  
  for (int i=0; i<sz; ++i) {
    double ai = fabs(A->data[i]);
    n = ai > n ? ai : n;
  }

  return n;
  
}
                               

int lm_der(int m, // # params
           int n, // # of residuals 
           double* p, // input: initial param vec; output: final param vec
           lm_res_func_t func, // function to optimize
           int maxiter, // maximum number of iterations 
           const lm_opts_t* opts, // may be null
           lm_info_t* info, //  may be null
           void* userdata) {

  lm_opts_t default_opts;

  if (!opts) {
    lm_opts_defaults(&default_opts);
    opts = &default_opts;
  }

  assert( opts );

  double lparam = opts->lparam ? opts->lparam : lm_lparam_default(opts->lfunc);

  // allocate space for Jacobian
  matd_t* J = matd_create(n,m);
  matd_t* x = matd_create(n,1);
  
  matd_t* W = opts->lfunc ? matd_create(n,1) : NULL;

  matd_t* JTWJ = matd_create(m,m);
  matd_t* JTWx = matd_create(m,1);

  matd_t* p_prev = matd_create_data(m, 1, p);
  matd_t* delta_p = matd_create(m, 1);

  matd_chol_t* chol = (matd_chol_t*)calloc(1, sizeof(matd_chol_t));

  double lambda = 1e5;
  
  double prev_loss = -1.0;

  int retval = LM_MAX_ITER;

  int iter;
  int stop_reason = LM_STOP_MAX_ITER;

  for (iter=0; iter<maxiter; ++iter) {

    ////////////////////////////////////////////////// 
    // get the residuals and jacobian

    func(m, n, p, x->data, J->data, userdata);

    //////////////////////////////////////////////////
    // compute loss
    
    double cur_loss = 0.0;

    if (opts->lfunc) {
      for (int i=0; i<n; ++i) {
        cur_loss += lm_loss(opts->lfunc, x->data[i], lparam, W->data+i);
      }
    } else {
      for (int i=0; i<n; ++i) {
        cur_loss += x->data[i]*x->data[i];
      }
      cur_loss *= 0.5;
    }

    //////////////////////////////////////////////////
    // check if improving

    if (prev_loss < 0.0) {
      if (info) {
        info->initial_loss = cur_loss;
      }
    } else if (cur_loss > prev_loss) {
      memcpy(p, p_prev->data, sizeof(double)*m);
      lambda *= 10.0;
      lambda = lambda < 1e8 ? lambda : 1e8;
      continue;
    }

    prev_loss = cur_loss;
    
    //////////////////////////////////////////////////
    // form JTWJ and JTWx

    transpose_product_inplace(J, W, J, JTWJ);
    transpose_product_inplace(J, W, x, JTWx);

    double JTWx_infnorm = inf_norm(JTWx);

    if (JTWx_infnorm < opts->JT_err_infnorm_tol) {
      retval = LM_SUCCESS;
      stop_reason = LM_STOP_JT_ERR;
      break;
    }

    //////////////////////////////////////////////////
    // add to diagonal and check for singularity

    double dmax = 0.0;
    
    for (int i=0; i<m; ++i) {
      double JTWJii = MATD_EL(JTWJ, i, i);
      JTWJii += lambda;
      //JTWJii *= (1.0 + lambda);
      dmax = (JTWJii > dmax) ? JTWJii : dmax;
      MATD_EL(JTWJ, i, i) = JTWJii;
    }

    lambda *= 0.5;
    lambda = lambda < 1e-5 ? 1e-5 : lambda;

    if (dmax < opts->JTJ_sing_tol) {
      retval = stop_reason = LM_SINGULAR_MATRIX;
      break;
    }
        
    //////////////////////////////////////////////////
    // do cholesky decompose/solve
    
    matd_chol_inplace(JTWJ, chol);

    if (!chol->is_spd) {
      retval = stop_reason = LM_SINGULAR_MATRIX;
      break;
    }
    
    matd_chol_solve_inplace(chol, JTWx, delta_p);

    //////////////////////////////////////////////////
    // update solution

    memcpy(p_prev->data, p, sizeof(double)*m);

    double relerr = inf_norm(delta_p) / inf_norm(p_prev);

    if (relerr < opts->delta_p_rel_infnorm_tol) {
      retval = LM_SUCCESS;
      stop_reason = LM_STOP_DELTA_P;
      break;
    }
    
    for (int i=0; i<m; ++i) {
      p[i] -= delta_p->data[i];
    }
    
  }

  matd_destroy(J);
  matd_destroy(x);
  matd_destroy(W);
  matd_destroy(JTWJ);
  matd_destroy(JTWx);
  matd_destroy(p_prev);
  matd_destroy(delta_p);
  matd_chol_destroy(chol);

  if (info) {
    info->final_loss = prev_loss;
    info->num_iterations = iter;
    info->stop_reason = stop_reason;
  }

  return retval;

}
