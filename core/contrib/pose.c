#include "pose.h"
#include "zarray.h"
#include "homography.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <float.h>


typedef struct matd_12 { int nrows, ncols; double data[12]; } matd_12_t;
typedef struct matd_9 { int nrows, ncols; double data[9]; } matd_9_t;
typedef struct matd_8 { int nrows, ncols; double data[8]; } matd_8_t;
typedef struct matd_6 { int nrows, ncols; double data[6]; } matd_6_t;
typedef struct matd_3 { int nrows, ncols; double data[3]; } matd_3_t;


matd_t* cross_mat(const double v[3]) {

    matd_t* X = matd_create(3, 3);

    MATD_EL(X, 0, 0) =  0;
    MATD_EL(X, 0, 1) = -v[2];
    MATD_EL(X, 0, 2) =  v[1];

    MATD_EL(X, 1, 0) =  v[2];
    MATD_EL(X, 1, 1) =  0;
    MATD_EL(X, 1, 2) = -v[0];

    MATD_EL(X, 2, 0) = -v[1];
    MATD_EL(X, 2, 1) =  v[0];
    MATD_EL(X, 2, 2) =  0;

    return X;
        
}

void cross_prod(const double a[3], const double b[3], double result[3]) {

    result[0] = a[1]*b[2] - a[2]*b[1];
    result[1] = a[2]*b[0] - a[0]*b[2];
    result[2] = a[0]*b[1] - a[1]*b[0];
    
}

// make a matrix to wrap a vector
matd_t* wrap_vec3(const double v[3]) {
    
    return matd_create_data(3, 1, v);
                             
}

void polar_decomp(const double rvec[3], double k[3], double* theta) {
    *theta = sqrt(rvec[0]*rvec[0] + rvec[1]*rvec[1] + rvec[2]*rvec[2]);
    for (int i=0; i<3; ++i) {
        k[i] = rvec[i] / *theta;
    }
}
    
matd_t* rvec_to_matrix(const double rvec[3]) {

    double k[3], theta;
    polar_decomp(rvec, k, &theta);

    double s = sin(theta);
    double c = cos(theta);

    matd_t* K = cross_mat(k);
    matd_t* K2 = matd_multiply(K, K);

    matd_scale_inplace(K, s);
    matd_scale_inplace(K2, (1-c));

    matd_add_inplace(K, K2);
    matd_destroy(K2);

    for (int i=0; i<3; ++i) {
        MATD_EL(K, i, i) += 1;
    }

    return K;
    
}

static void accum_vec(double a[3], double alpha, double b[3]) {
    for (int i=0; i<3; ++i) {
        a[i] += alpha * b[i];
    }
}

void rotate_vector(const double rvec[3],
                   const double v[3],
                   double Rv[3],
                   matd_t** J) {

    double k[3], theta;
    polar_decomp(rvec, k, &theta);

    double s = sin(theta);
    double c = cos(theta);

    if (Rv) {
        
        Rv[0] = v[0];
        Rv[1] = v[1];
        Rv[2] = v[2];

        double kxv[3], kxkxv[3];
        
        cross_prod(k, v, kxv);
        cross_prod(k, kxv, kxkxv);
        
        accum_vec(Rv, s, kxv);
        accum_vec(Rv, (1-c), kxkxv);


    }

    if (!J) { return; }

    double w = c - 1;
    
    double kx = k[0];
    double ky = k[1];
    double kz = k[2];

    double px = v[0];
    double py = v[1];
    double pz = v[2];

    matd_12_t Afoo = {
        3, 4,
        {
            
            -w*(ky*py + kz*pz),
            -kx*py*w + 2*ky*px*w + pz*s,
            -kx*pz*w + 2*kz*px*w - py*s,
            -px*s*(ky*ky + kz*kz) + py*(-c*kz + kx*ky*s) + pz*(c*ky + kx*kz*s),
            
            2*kx*py*w - ky*px*w - pz*s,
            -w*(kx*px + kz*pz),
            -ky*pz*w + 2*kz*py*w + px*s,
            px*(c*kz + kx*ky*s) - py*s*(kx*kx + kz*kz) - pz*(c*kx - ky*kz*s),
            
            2*kx*pz*w - kz*px*w + py*s,
            2*ky*pz*w - kz*py*w - px*s,
            -w*(kx*px + ky*py),
            px*(-c*ky + kx*kz*s) + py*(c*kx + ky*kz*s) - pz*s*(kx*kx + ky*ky),
            
        }
    };

    matd_12_t Bfoo = {
        4, 3, 
        {
            (ky*ky + kz*kz)/theta,
            -kx*ky/theta,
            -kx*kz/theta,
            
            -kx*ky/theta,
            (kx*kx + kz*kz)/theta,
            -ky*kz/theta,
            
            -kx*kz/theta,
            -ky*kz/theta,
            (kx*kx + ky*ky)/theta,
            
            kx,
            ky,
            kz,
        }
    };

    const matd_t* A = (const matd_t*)&Afoo;
    const matd_t* B = (const matd_t*)&Bfoo;

    *J = matd_multiply(A, B);

}

void rvec_from_matrix(const matd_t* R, double rvec[3]) {

    double q[4];
    
    quaternion_from_matrix(R, q);
    rvec_from_quaternion(q, rvec);
    
}

void quaternion_from_matrix(const matd_t* R, double q[4]) {

    assert( (R->nrows == 3 && R->ncols == 3) || (R->nrows == 4 && R->ncols == 4) );

    double r11 = MATD_EL(R, 0, 0);
    double r12 = MATD_EL(R, 0, 1);
    double r13 = MATD_EL(R, 0, 2);

    double r21 = MATD_EL(R, 1, 0);
    double r22 = MATD_EL(R, 1, 1);
    double r23 = MATD_EL(R, 1, 2);

    double r31 = MATD_EL(R, 2, 0);
    double r32 = MATD_EL(R, 2, 1);
    double r33 = MATD_EL(R, 2, 2);

    // https://en.wikipedia.org/wiki/Rotation_matrix#Quaternion
    double t = r11 + r22 + r33;

    if (t > 0.) {

        double r = sqrt(1 + t);
        double s = 0.5/r;
            
        q[3] = 0.5 * r;
        
        q[0] = (r32 - r23) * s;
        q[1] = (r13 - r31) * s;
        q[2] = (r21 - r12) * s;
        
    } else {

        double r = sqrt(1 + r11 - r22 - r33);
        double s = 0.5 / r;

        q[3] = (r32 - r23) * s;
        
        q[0] = 0.5 * r;
        q[1] = (r12 + r21) * s;
        q[2] = (r31 + r13) * s;
        
    }

    // prefer smaller angles for angle-axis representation
    if (q[3] < 0) {
        for (int i=0; i<4; ++i) { q[i] = -q[i]; }
    }

}

void rvec_from_quaternion(const double qorig[4], double rvec[3]) {

    const double* q = qorig;
    double tmp[4];

    if (q[3] < -1 || q[3] > 1) {

        printf("had ta normalize!\n");

        // normalize
        double norm2 = 0;
        for (int i=0; i<4; ++i) { norm2 += q[i]; }
        
        double l = sqrt(norm2);

        for (int i=0; i<4; ++i) { tmp[i] = qorig[i] / l; }
        q = tmp;
        
    }

    double k = 2 * acos(q[3]) / sqrt(1 - q[3]*q[3]);

    for (int i=0; i<3; ++i) {
        rvec[i] = k*q[i];
    }

}

void project_points(double fx, double fy, double cx, double cy,
                    double tagsize,
                    const double rvec[3],
                    const double tvec[3],
                    double corners_reproj[][2], // 4x2
                    matd_t** Jptr) {

    const double corners_raw[4][3] = {
        { -0.5*tagsize, -0.5*tagsize, 0 },
        {  0.5*tagsize, -0.5*tagsize, 0 },
        {  0.5*tagsize,  0.5*tagsize, 0 },
        { -0.5*tagsize,  0.5*tagsize, 0 }
    };

    if (Jptr) { *Jptr = matd_create(8, 6); }
    
    matd_9_t dpi_dgi = {
        3, 3, {
            fx, 0, cx,
            0, fy, cy,
            0, 0, 1
        }
    };
    
    for (int i=0; i<4; ++i) {
        
        const double* vi = corners_raw[i];
        double gi[3];
        
        // g = R(r)*v 
        matd_t* dgi_dr;
        rotate_vector(rvec, vi, gi, Jptr ? &dgi_dr : NULL);

        // g = R(r)*v + t
        for (int j=0; j<3; ++j) { gi[j] += tvec[j]; }

        // p = K*g
        double pi[3] = { gi[0] * fx + gi[2] * cx,
                         gi[1] * fy + gi[2] * cy,
                         gi[2] };

        // q = perspective_divide(p)
        double* qi = corners_reproj[i];

        qi[0] = pi[0]/pi[2];
        qi[1] = pi[1]/pi[2];

        if (Jptr) { 

            matd_6_t dqi_dpi = {
                2, 3, {
                    1/pi[2], 0, -pi[0]/(pi[2]*pi[2]),
                    0, 1/pi[2], -pi[1]/(pi[2]*pi[2])
                }
            };

            matd_t* dqi_dgi = matd_multiply((const matd_t*)&dqi_dpi,
                                            (const matd_t*)&dpi_dgi);

            // dqi_dr = dqi_dpi * dpi_dgi * dgi_dr
            //        = dqi_dgi

            matd_t* dqi_dr = matd_multiply(dqi_dgi, dgi_dr);
            matd_t* dqi_dt = dqi_dgi; // * I

            for (int k=0; k<2; ++k) {
                for (int j=0; j<3; ++j) {
                    MATD_EL(*Jptr, 2*i+k, j+0) = MATD_EL(dqi_dr, k, j);
                    MATD_EL(*Jptr, 2*i+k, j+3) = MATD_EL(dqi_dt, k, j);
                }
            }

            matd_destroy(dqi_dr);
            matd_destroy(dqi_dgi);
            matd_destroy(dgi_dr);
            
        }
        
    }


}

double reprojection_error(const double corners_meas[][2],
                          const double corners_reproj[][2],
                          const matd_t* Jrt,
                          double rtgrad[6]) {

    matd_8_t err = { 8, 1, { 0 } };
    double errsum = 0;


    for (int i=0; i<4; ++i) {
        for (int k=0; k<2; ++k) {

            int row = 2*i + k;

            double ei = 0.5 * (corners_reproj[i][k] - corners_meas[i][k]);
            errsum += ei * ei;

            err.data[row] = ei;

        }
    }

    if (rtgrad && Jrt) {
        matd_t* g = matd_op("M'*M", Jrt, (const matd_t*)&err);
        memcpy(rtgrad, g->data, 6*sizeof(double));
        matd_destroy(g);
    }

    return  errsum;

}

double reprojection_objective(const double corners_meas[][2], 
                              double fx, double fy, double kx, double ky,
                              double tagsize,
                              const double rvec[3],
                              const double tvec[3],
                              double rtgrad[6],
                              matd_t** Jptr) {

    double corners_reproj[4][2];

    matd_t* Jrt = NULL;

    project_points(fx, fy, kx, ky, tagsize,
                   rvec, tvec,
                   corners_reproj,
                   &Jrt);

    double retval = reprojection_error(corners_meas, corners_reproj,
                                       Jrt, rtgrad);

    if (Jptr) {
        *Jptr = Jrt;
    } else {
        matd_destroy(Jrt);
    }

    return retval;


}

matd_t* pose_from_homography(const matd_t* H,
                             double fx, double fy, double cx, double cy,
                             double tagsize,
                             double z_sign,
                             const double corners_meas[][2],
                             double* initial_error,
                             double* final_error) {

    matd_t* M = homography_to_pose(H, fx, fy, cx, cy);

    if (MATD_EL(M, 2, 3) * z_sign < 0) {

        for (int i=0; i<3; ++i) {
            MATD_EL(M, i, 0) *= -1;
            MATD_EL(M, i, 1) *= -1;
            MATD_EL(M, i, 3) *= -1;
        }

    }

    for (int i=0; i<3; ++i) {
        MATD_EL(M, i, 3) *= 0.5*tagsize;
    }

    
    if (!corners_meas) { return M; }
        
    double rvec[3], tvec[3];
        
    mat4_to_rvec_tvec(M, rvec, tvec);

    double best_e = DBL_MAX;
    
    matd_6_t g = { 6, 1, { 0 } };


    double best_rvec[3], best_tvec[3];

    const double LMAX = 1e5;
    const double LMIN = 1e-7;
    const double STEP_TOL = 1e-12;
    const int MAX_ITER = 100;
        
    double lambda = LMAX;
    int done = 0;

    memcpy(best_rvec, rvec, sizeof(rvec));
    memcpy(best_tvec, tvec, sizeof(tvec));
        
    for (int iter=0; iter<MAX_ITER; ++iter) {

        matd_t* J;

        double e = reprojection_objective(corners_meas,
                                          fx, fy, cx, cy, tagsize,
                                          rvec, tvec, g.data,
                                          done ? NULL : &J);

        //printf("objective at iter %3d is %12f\n", iter, e);

        if (e < best_e) {
            best_e = e;
            memcpy(best_rvec, rvec, sizeof(rvec));
            memcpy(best_tvec, tvec, sizeof(tvec));
            lambda *= 0.5;
            if (lambda < LMIN) { lambda = LMIN; }
            //printf("better than previous best of %g, decreased to lambda=%g\n", best_e, lambda);
        } else {
            memcpy(rvec, best_rvec, sizeof(rvec));
            memcpy(tvec, best_tvec, sizeof(tvec));
            lambda *= 10.0;
            if (lambda > LMAX) { lambda = LMAX; }
            //printf("worse than previous best of %g, increased to lambda=%g\n", best_e, lambda);
        }

        if (iter == 0 && initial_error) {
            *initial_error = best_e;
        }
        
        if (done) {
            break;
        }

        matd_t* JTJ = matd_op("M'*M", J, J);

        for (int i=0; i<6; ++i) {
            MATD_EL(JTJ, i, i) += lambda;
        }

        matd_t* step = matd_solve(JTJ, (matd_t*)&g);

        double stotal = 0;
        for (int i=0; i<6; ++i) {
            stotal += step->data[i] * step->data[i];
        }

        for (int i=0; i<3; ++i) {
            rvec[i] -= step->data[i+0];
            tvec[i] -= step->data[i+3];
        }

        matd_destroy(J);
        matd_destroy(JTJ);
        matd_destroy(step);

        if (stotal < STEP_TOL) {
            done = 1;
        }
            
    }

    if (final_error) { *final_error = best_e; }

    matd_destroy(M);
           
    return mat4_from_rvec_tvec(rvec, tvec);

}

void mat4_to_rvec_tvec(const matd_t* M,
                       double rvec[3],
                       double tvec[3]) {

    for (int i=0; i<3; ++i) {
        tvec[i] = MATD_EL(M, i, 3);
    }

    rvec_from_matrix(M, rvec);

}

matd_t* mat4_from_rvec_tvec(const double rvec[3],
                            const double tvec[3]) {

    matd_t* R = rvec_to_matrix(rvec);

    matd_t* M = matd_create(4, 4);

    MATD_EL(M, 3, 3) = 1;

    for (int i=0; i<3; ++i) {
        MATD_EL(M, i, 3) = tvec[i];
        MATD_EL(M, 3, i) = 0;
        for (int j=0; j<3; ++j) {
            MATD_EL(M, i, j) = MATD_EL(R, i, j);
        }
    }

    matd_destroy(R);

    return M;

}
