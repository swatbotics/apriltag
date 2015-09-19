#ifndef _LM_H_
#define _LM_H_

#ifdef __cplusplus
extern "C" {
#endif 

typedef void (*lm_res_func_t)(int m, // # params (unknowns)
                              int n, // # residuals 
                              const double* p, // # m-by-1 vector input of params
                              double* x, // n-by-1 vector output of residuals
                              double* J, // n-by-m row-major matrix of partial derivs so that J(row,col) = J[m*row + col]
                              void* userdata); //

typedef enum lm_loss_func {
  LM_LOSS_L2=0,
  LM_LOSS_L1,
  LM_LOSS_L1_MINUS_L2,
  LM_LOSS_LEAST_POWER,
  LM_LOSS_FAIR,
  LM_LOSS_HUBER,
  LM_LOSS_TUKEY,
} lm_loss_func_t;

enum {
  LM_NUM_LOSS_FUNC = LM_LOSS_TUKEY+1
};

double lm_lparam_default(lm_loss_func_t lfunc);

const char* lm_loss_func_name(lm_loss_func_t lfunc);

typedef struct lm_opts {
  double JTJ_sing_tol; // tolerance on 
  double JT_err_infnorm_tol;
  double delta_p_rel_infnorm_tol;
  double loss_tol;
  lm_loss_func_t lfunc;
  double lparam; // parameter to loss function
} lm_opts_t;

typedef struct lm_info {
  double initial_loss;
  double final_loss;
  int    num_iterations;
  int    stop_reason;
} lm_info_t;

enum lm_result {
  LM_SUCCESS = 0,
  LM_MAX_ITER = -1,
  LM_SINGULAR_MATRIX = -2
};

enum lm_stop_criterion {
  LM_STOP_JT_ERR = 1,
  LM_STOP_DELTA_P = 2,
  LM_STOP_LOSS = 3,
  LM_STOP_MAX_ITER = LM_MAX_ITER,
  LM_STOP_SINGULAR_MATRIX = LM_SINGULAR_MATRIX
};

// can handle either a result or a stop criterion 
const char* lm_result_to_string(int result);

void lm_check_residual(int m,
                       int n,
                       double* p,
                       lm_res_func_t func,
                       void* userdata);

void lm_check_loss(int m,
                   int n,
                   double* p,
                   lm_res_func_t func,
                   void* userdata,
                   lm_loss_func_t loss,
                   double lparam);

void lm_opts_defaults(lm_opts_t* opts);

int lm_der(int m, // # of params (unknowns)
           int n, // # of residuals 
           double* p, // input: initial param vec; output: final param vec
           lm_res_func_t func, // function to optimize
           int maxiter, // maximum number of iterations 
           const lm_opts_t* opts, // may be null
           lm_info_t* info, //  may be null
           void* userdata); // passed blindly to func

#ifdef __cplusplus
}
#endif 

 
#endif
