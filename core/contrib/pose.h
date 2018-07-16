#ifndef _ROTATION_H_
#define _ROTATION_H_

#include "matd.h"

#ifdef __cplusplus
extern "C" {
#endif

// create skew-symmetric cross product matrix for vector
matd_t* cross_mat(const double v[3]);

// take cross product of a and b and place it into result
void cross_prod(const double a[3], const double b[3], double result[3]);

// make a matrix to wrap a vector 
matd_t* wrap_vec3(const double v[3]);

// use Rodrigues' formula to compute a 3x3 rotation matrix
matd_t* rvec_to_matrix(const double rvec[3]);

// converts to quaternion internally and goes from there
void rvec_from_matrix(const matd_t* R, double rvec[3]);

// we use the scalar-element-first convention (as in Diebel)
void quaternion_from_matrix(const matd_t* R, double q[4]);
void rvec_from_quaternion(const double q[4], double rvec[3]);

// rotate vector v by rotation given in rvec, and return the
// jacobian of the result with respect to rvec (or NULL if
// compute_derivative not set)
void rotate_vector(const double rvec[3],
                   const double v[3],
                   double Rv[3],
                   matd_t** J);

matd_t* pose_from_homography(const matd_t* H,
                             double fx, double fy, double cx, double cy,
                             double tagsize,
                             double z_sign,
                             const double corners_meas[][2],
                             double* initial_error,
                             double* final_error);


void project_points(double fx, double fy, double cx, double cy,
                    double tagsize,
                    const double rvec[3],
                    const double tvec[3],
                    double corners_proj[][2], // 4x2
                    matd_t** J);
                    

double reprojection_error(const double corners_meas[][2],   // 4x2
                          const double corners_reproj[][2], // 4x2
                          const matd_t* Jrt,
                          double rtgrad[6]);

double reprojection_objective(const double corners_meas[][2], // 
                              double fx, double fy, double cx, double cy,
                              double tagsize,
                              const double rvec[3],
                              const double tvec[3],
                              double rtgrad[6],
                              matd_t** J);

void mat4_to_rvec_tvec(const matd_t* M,
                       double rvec[3],
                       double tvec[3]);

matd_t* mat4_from_rvec_tvec(const double rvec[3],
                            const double tvec[3]);
                         

#ifdef __cplusplus
}
#endif

#endif
