#include "pose.h"
#include "zarray.h"
#include "homography.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#ifdef NDEBUG
#undef NDEBUG
#endif

#include <assert.h>


const char* MAT_FMT = "%12f, ";
double EPS = 1e-5;

const double fx = 764, fy = 764, cx = 408, cy = 306;

const double rvec[3] = { 0.5, 0.2, 0.02 };
const double tvec[3] = { 0.1, -0.05, 1.0 };

const double tagsize = 0.15;

const double corners_meas[4][2] = {
    { 428, 212 },
    { 547, 212 },
    { 540, 320 },
    { 427, 315 }
};



int all_close(const double* a,
              const double* b,
              int nr, int nc, double tol) { 

    int sz = nr*nc;
    for (int i=0; i<sz; ++i) {
        if (!a[i]) {
            if (fabs(b[i]) > tol) { return 0; }
        } else {
            double relerr = fabs(a[i]-b[i])/a[i];
            if (relerr > tol) { return 0; }
        }
    }

    return 1;

}


#define verify(a, b, nr, nc) _verify(a, b, nr, nc, #a, #b, __FILE__, __LINE__, __FUNCTION__)

void _verify(const double* a, const double* b,
             int nr, int nc,
             const char* aname, const char* bname,
             const char* file,
             int line,
             const char* fn) {

    int ok = all_close(a, b, nr, nc, EPS);

    if (ok) {
        printf("%s = %s: pass\n\n", aname, bname);
    } else {
        fprintf(stderr, "%s:%d %s != %s in %s\n",
                file, line, aname, bname, fn);
        exit(1);
    }

}

double rand_double() {
    return rand() / (double)RAND_MAX;
}

void random_vec3(double v[3], double r) {

    double r2 = r*r;

    while (1) {
        double l2 = 0;
        for (int i=0; i<3; ++i) {
            v[i] = r * (rand_double() * 2 - 1);
            l2 += v[i]*v[i];
        }
        if (l2 <= r2) { break; }
    }

}


void print_mat(const double* src, int nrows, int ncols, const char* fmt) {
    int cnt = 0;
    for (int i=0; i<nrows; ++i) {
        for (int j=0; j<ncols; ++j) {
            printf(fmt, src[cnt++]);
        }
        printf("\n");
    }
}

void print_vec3(const double* v) {
    print_mat(v, 1, 3, MAT_FMT);
}

void test_basics() {

    const double s = 0.1;
    
    const double rvec[3] = { s*1, s*2, s*3 };
    const double v[3] = { 0.4, 0.5, 0.6 };

    matd_t* R = rvec_to_matrix(rvec);

    printf("this should be a rotation matrix:\n");
    matd_print(R, MAT_FMT);
    printf("\n");

    matd_t* I = matd_op("M*M'", R, R);

    printf("this should be the identity:\n");
    matd_print(I, MAT_FMT);
    printf("\n");

    matd_t* vmat = wrap_vec3(v);
    matd_t* Rvmat = matd_multiply(R, vmat);

    printf("Rv        = ");
    print_vec3(Rvmat->data);

    double Rv[3];
    matd_t* J;

    rotate_vector(rvec, v, Rv, &J);

    printf("Rv (mine) = ");
    print_vec3(Rv);
    verify(Rvmat->data, Rv, 3, 1);
    

    matd_t* Jn = matd_create(3, 3);
    const double h = 1e-4;

    for (int i=0; i<3; ++i) {
        
        double tmp[3];
        double Rv0[3], Rv1[3];

        memcpy(tmp, (const double*)rvec, sizeof(rvec));
        
        tmp[i] = rvec[i] - h;
        rotate_vector(tmp, v, Rv0, 0);

        tmp[i] = rvec[i] + h;
        rotate_vector(tmp, v, Rv1, 0);
        
        for (int j=0; j<3; ++j) {
            MATD_EL(Jn, j, i) = 0.5 * (Rv1[j] - Rv0[j]) / h;
        }
        
        
    }

    printf("J =\n");
    matd_print(J, MAT_FMT);
    
    printf("Jn =\n");
    matd_print(Jn, MAT_FMT);
    verify(J->data, Jn->data, 3, 3);
    
    double rvec2[3];

    rvec_from_matrix(R, rvec2);

    printf("rvec  = ");
    print_vec3(rvec);
    printf("rvec2 = ");
    print_vec3(rvec2);

    verify(rvec, rvec2, 3, 1);

    matd_destroy(R);
    matd_destroy(I);
    matd_destroy(vmat);
    matd_destroy(Rvmat);
    matd_destroy(J);
    matd_destroy(Jn);

    printf("%s: PASS\n\n", __FUNCTION__);

}



void test_rvec_sampling() {

    srand(0xDEFACED);

    for (int i=0; i<1000; ++i) {

        double rvec[3];
        random_vec3(rvec, M_PI);

        matd_t* R = rvec_to_matrix(rvec);

        double rvec2[3];

        rvec_from_matrix(R, rvec2);

        matd_destroy(R);

        printf("rvec  = ");
        print_vec3(rvec);
        
        printf("rvec2 = ");
        print_vec3(rvec2);

        verify(rvec, rvec2, 3, 1);
        
    }

    printf("%s: PASS\n\n", __FUNCTION__);
    

}

void test_proj() {


    double corners[4][2];

    matd_t* J;
    project_points(fx, fy, cx, cy, tagsize, rvec, tvec, corners, &J);

    matd_t* R = rvec_to_matrix(rvec);
    
    matd_t* K =
        matd_create_data(3, 3, (double[]) { fx, 0, cx, 0, fy, cy, 0, 0, 1 });
                    

    matd_t* v = matd_create(3, 1);
    double corners2[4][2];

    for (int i=0; i<4; ++i) {
        
        MATD_EL(v, 0, 0) = (i == 0 || i == 3) ? -0.5*tagsize : 0.5*tagsize;
        MATD_EL(v, 1, 0) = (i == 0 || i == 1) ? -0.5*tagsize : 0.5*tagsize;
        MATD_EL(v, 2, 0) = 0;
        
        matd_t* g = matd_multiply(R, v);

        for (int j=0; j<3; ++j) {
            MATD_EL(g, j, 0) += tvec[j];
        }
        
        matd_t* p = matd_multiply(K, g);

        for (int k=0; k<2; ++k) {
            corners2[i][k] = MATD_EL(p, k, 0) / MATD_EL(p, 2, 0);
        }

        matd_destroy(g);
        matd_destroy(p);
        
    }

    printf("corners:\n");
    print_mat(corners[0], 4, 2, MAT_FMT);
    
    printf("corners2:\n");
    print_mat(corners2[0], 4, 2, MAT_FMT);

    verify(corners[0], corners2[0], 4, 2);


    matd_t* Jn = matd_create(8, 6);

    const float h = 1e-4;

    for (int i=0; i<6; ++i) {

        double c1[4][2], c0[4][2];

        double r0[3], t0[3], r1[3], t1[3];

        memcpy(r0, rvec, sizeof(r0));
        memcpy(r1, rvec, sizeof(r1));

        memcpy(t0, tvec, sizeof(t0));
        memcpy(t1, tvec, sizeof(t1));

        if (i < 3) {
            r0[i] -= h;
            r1[i] += h;
        } else {
            t0[i-3] -= h;
            t1[i-3] += h;
        }
        
        project_points(fx, fy, cx, cy, tagsize, r0, t0, c0, 0);
        project_points(fx, fy, cx, cy, tagsize, r1, t1, c1, 0);

        const double* c1flat = c1[0];
        const double* c0flat = c0[0];

        for (int j=0; j<8; ++j) {
            MATD_EL(Jn, j, i) = (c1flat[j] - c0flat[j]) / (2*h);
        }
        
    }


    printf("J:\n");
    matd_print(J, MAT_FMT);
    
    printf("Jn:\n");
    matd_print(Jn, MAT_FMT);

    verify(J->data, Jn->data, 8, 6);

    matd_destroy(R);
    matd_destroy(v);
    matd_destroy(K);
    matd_destroy(J);
    matd_destroy(Jn);

    printf("%s: PASS\n\n", __FUNCTION__);
    
}

void test_reprojection_error() {
     
    double corners_reproj[4][2];

    matd_t* J;
    project_points(fx, fy, cx, cy, tagsize,
                   rvec, tvec, corners_reproj,
                   &J);

    double g[6];

    reprojection_error(corners_meas,
                       corners_reproj,
                       J, g);

    const double h = 1e-5;

    double gn[6];

    for (int i=0; i<6; ++i) {
        
        double c[4][2];

        double r0[3], t0[3], r1[3], t1[3];

        memcpy(r0, rvec, sizeof(r0));
        memcpy(r1, rvec, sizeof(r1));

        memcpy(t0, tvec, sizeof(t0));
        memcpy(t1, tvec, sizeof(t1));

        if (i < 3) {
            r0[i] -= h;
            r1[i] += h;
        } else {
            t0[i-3] -= h;
            t1[i-3] += h;
        }

        project_points(fx, fy, cx, cy, tagsize, r0, t0, c, 0);
        double e0 = reprojection_error(corners_meas, c, NULL, NULL);

        project_points(fx, fy, cx, cy, tagsize, r1, t1, c, 0);
        double e1 = reprojection_error(corners_meas, c, NULL, NULL);
        
        gn[i] = (e1 - e0) / (2*h);
        
    }

    printf("g:  ");
    print_mat(g, 1, 6, MAT_FMT);

    printf("gn: ");
    print_mat(gn, 1, 6, MAT_FMT);

    verify(g, gn, 1, 6);

    matd_destroy(J);

    printf("%s: PASS\n\n", __FUNCTION__);
    
}


void test_reprojection_objective() {

    double g[6];

    reprojection_objective(corners_meas,
                           fx, fy, cx, cy,
                           tagsize,
                           rvec, tvec, g, NULL);

    const double h = 1e-5;

    double gn[6];

    for (int i=0; i<6; ++i) {

        double r0[3], t0[3], r1[3], t1[3];

        memcpy(r0, rvec, sizeof(r0));
        memcpy(r1, rvec, sizeof(r1));

        memcpy(t0, tvec, sizeof(t0));
        memcpy(t1, tvec, sizeof(t1));

        if (i < 3) {
            r0[i] -= h;
            r1[i] += h;
        } else {
            t0[i-3] -= h;
            t1[i-3] += h;
        }

        double e0 = reprojection_objective(corners_meas,
                                           fx, fy, cx, cy,
                                           tagsize,
                                           r0, t0, NULL, NULL);

        double e1 = reprojection_objective(corners_meas,
                                           fx, fy, cx, cy,
                                           tagsize,
                                           r1, t1, NULL, NULL);

        gn[i] = (e1 - e0) / (2*h);
        
    }

    printf("g:  ");
    print_mat(g, 1, 6, MAT_FMT);

    printf("gn: ");
    print_mat(gn, 1, 6, MAT_FMT);

    verify(g, gn, 1, 6);
    

    printf("%s: PASS\n\n", __FUNCTION__);
    
}

matd_t* homography_from_corners(const double corners[][2]) {

    zarray_t* correspondences = zarray_create(sizeof(float[4]));
    
    for (int i=0; i<4; ++i) {
        
        float corr[4];
        
        corr[0] = (i==0 || i==3) ? -1 : 1;
        corr[1] = (i==0 || i==1) ? -1 : 1;
        corr[2] = corners[i][0];
        corr[3] = corners[i][1];
        
        zarray_add(correspondences, &corr);

    }


    matd_t* H = homography_compute(correspondences, HOMOGRAPHY_COMPUTE_FLAG_SVD);

    zarray_destroy(correspondences);

    return H;

}
    

void test_pose_from_homograpy() {

    double corners[4][2];

    project_points(fx, fy, cx, cy, tagsize,
                   rvec, tvec, corners,
                   0);

    matd_t* H = homography_from_corners(corners);
    
    printf("H:\n");
    matd_print(H, MAT_FMT);
    printf("\n");

    matd_t* M = pose_from_homography(H, fx, fy, cx, cy, tagsize, 1.0,
                                     NULL, NULL, NULL);

    double rvec2[3], tvec2[3];

    mat4_to_rvec_tvec(M, rvec2, tvec2);

    printf("rvec:  ");
    print_vec3(rvec);
    printf("rvec2:  ");
    print_vec3(rvec2);
    verify(rvec, rvec2, 3, 1);

    printf("tvec:  ");
    print_vec3(tvec);
    printf("tvec2:  ");
    print_vec3(tvec2);
    verify(tvec, tvec2, 3, 1);
    
    matd_destroy(H);
    matd_destroy(M);

    printf("%s: PASS\n\n", __FUNCTION__);
    
}

void test_pose_from_homograpy_refine_contrived() {

    double corners[4][2];

    project_points(fx, fy, cx, cy, tagsize,
                   rvec, tvec, corners,
                   0);

    // initialize homography with wrong points
    matd_t* H_bad = homography_from_corners(corners_meas);

    double e_bad, e_good;

    // get pose for this - should disagree with "correct" pose
    matd_t* M_bad = pose_from_homography(H_bad, fx, fy, cx, cy, tagsize, 1.0, NULL, NULL, NULL);
    
    // get pose for correct points
    matd_t* M_good = pose_from_homography(H_bad, fx, fy, cx, cy, tagsize, 1.0, corners, &e_bad, &e_good);

    double rvec2[3], tvec2[3], rvec3[3], tvec3[3];

    mat4_to_rvec_tvec(M_bad, rvec2, tvec2);
    mat4_to_rvec_tvec(M_good, rvec3, tvec3);

    double e_bad2 = reprojection_objective(corners,
                                           fx, fy, cx, cy, tagsize,
                                           rvec2, tvec2, NULL, NULL);

    double e_good2 = reprojection_objective(corners,
                                            fx, fy, cx, cy, tagsize,
                                            rvec3, tvec3, NULL, NULL);

    verify(&e_bad, &e_bad2, 1, 1);
    verify(&e_good, &e_good2, 1, 1);

    const double zero = 0;

    printf("e_bad=%g, e_good=%g\n", e_bad, e_good);
    verify(&zero, &e_good, 1, 1);
    
    printf("rvec:  ");
    print_vec3(rvec);
    printf("rvec2:  ");
    print_vec3(rvec2);
    printf("rvec3:  ");
    print_vec3(rvec3);
    verify(rvec, rvec3, 3, 1);

    printf("tvec:  ");
    print_vec3(tvec);
    printf("tvec2:  ");
    print_vec3(tvec2);
    printf("tvec3:  ");
    print_vec3(tvec3);
    verify(tvec, tvec3, 3, 1);

    matd_destroy(H_bad);
    matd_destroy(M_good);
    matd_destroy(M_bad);

    printf("%s: PASS\n\n", __FUNCTION__);
    
}

void test_pose_from_homograpy_refine_plausible() {


    // initialize homography with wrong points
    matd_t* H_bad = homography_from_corners(corners_meas);

    double e_bad, e_bad2, e_ok, e_ok2;

    matd_t* M_bad = pose_from_homography(H_bad, fx, fy, cx, cy, tagsize, 1.0, NULL, NULL, NULL);

    matd_t* M_ok = pose_from_homography(H_bad, fx, fy, cx, cy, tagsize, 1.0, corners_meas, &e_bad, &e_ok);

    double rvec[3], tvec[3], rvec2[3], tvec2[3];
    
    mat4_to_rvec_tvec(M_bad, rvec, tvec);
    mat4_to_rvec_tvec(M_ok, rvec2, tvec2);
    
    e_bad2 = reprojection_objective(corners_meas,
                                    fx, fy, cx, cy, tagsize,
                                    rvec, tvec, NULL, NULL);
    
    e_ok2 = reprojection_objective(corners_meas,
                                   fx, fy, cx, cy, tagsize,
                                   rvec2, tvec2, NULL, NULL);
    
    verify(&e_ok, &e_ok2, 1, 1);
    
    verify(&e_bad, &e_bad2, 1, 1);

    if (e_ok >= e_bad) {
        fprintf(stderr, "%s:%d error: expect e_ok < e_bad in %s\n",
                __FILE__, __LINE__, __FUNCTION__);
        exit(1);
    }

    printf("%s: PASS\n\n", __FUNCTION__);

    matd_destroy(H_bad);
    matd_destroy(M_bad);
    matd_destroy(M_ok);
    
    
}


int main(int argc, char** argv) {

    test_basics();
    test_rvec_sampling();
    test_proj();
    test_reprojection_error();
    test_reprojection_objective();
    test_pose_from_homograpy();
    test_pose_from_homograpy_refine_contrived();
    test_pose_from_homograpy_refine_plausible();
    
    return 0;


}
