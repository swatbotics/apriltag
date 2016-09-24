#include "apriltag_opencv.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include "apriltag_vis.h"

Mat8uc1 makeImage(const apriltag_detection_t* det) {

  image_u8_t* image = apriltag_vis_texture(det);
  Mat8uc1 rval = im2cv(image).clone();
  image_u8_destroy(image);
  return rval;

}

Mat64fc1 getWarp(const apriltag_detection_t* detection) {

  matd_t* w = apriltag_vis_warp(detection);
  Mat64fc1 rval = Mat64fc1(3, 3, w->data).clone();
  matd_destroy(w);
  return rval;

}

cv::Mat detectionsImage(zarray_t* detections,
                        const cv::Size& size,
                        int type) {

  cv::Mat dst(size, type);

  image_u8_t* image = image_u8_create(size.width, size.height);
  cv::Mat im = im2cv(image);

  apriltag_vis_detections(detections, image);

  if (im.depth() != dst.depth()) {
    cv::Mat i2;
    double scl = 1.0;
    if (dst.depth() == CV_32F || dst.depth() == CV_64F) {
      scl = 1.0/255;
    } 
    im.convertTo(i2, dst.depth(), scl);
    im = i2;
  }

  if (im.channels() < dst.channels()) {
    cv::Mat i2;
    cv::cvtColor(im, i2, cv::COLOR_GRAY2RGB);
    im = i2;
  }

  im.copyTo(dst);

  return dst;

}


cv::Scalar color_from_hue(double h) {
  
  cv::Scalar rval;
  
  h -= floor(h);
  h *= 6.0;

  for (int i=0; i<3; ++i) {
    double ci = fmod(h + 2.0*i, 6.0);
    if (ci < 0.0 || ci > 6.0) {
      std::cerr << "ci = " << ci << "\n";
      exit(1);
    }
    ci = std::min(ci, 4.0-ci);
    ci = std::max(0.0, std::min(ci, 1.0));
    rval[i] = ci*255;
  }

  return rval;

}

cv::Scalar random_color() {

  double h = cv::theRNG().uniform(0.0, 1.0);
  return color_from_hue(h);

}

cv::Point scale_point(int x, int y, int u) {
  int b = u/2;
  return cv::Point(x*u + b, y*u + b);
}

cv::Point scale_point(const cv::Point2f p, int u, int shift) {
  int b = u/2;
  u *= (1 << shift);
  return cv::Point(p.x*u + b + 0.5, p.y*u + b + 0.5);
}

void contour_to_points(const zarray_t* cpoints,
                       std::vector<cv::Point>& points,
                       int upscale) {

  points.clear();

  for (int i=0; i<zarray_size(cpoints); ++i) {
    const contour_point_t* p;
    zarray_get_volatile(cpoints, i, &p);
    points.push_back(scale_point(p->x, p->y, upscale));
  }

}

void arrow(cv::Mat image,
           const cv::Point2f& p0,
           const cv::Point2f& p1,
           const cv::Scalar& color,
           int u) {

  cv::Point2f d10 = p1-p0;
  d10 *= 1.0 / sqrt(d10.x*d10.x + d10.y*d10.y);

  cv::Point2f n(d10.y, -d10.x);

  int shift = 4;
  
  cv::Point a = scale_point(p0, u, shift);
  cv::Point b = scale_point(p1, u, shift);
  cv::Point c = scale_point(p1 - 4*d10 - 2*n, u, shift);
  cv::Point d = scale_point(p1 - 4*d10 + 2*n, u, shift);

  cv::line(image, a, b, color, 1, CV_AA, shift);
  cv::line(image, b, c, color, 1, CV_AA, shift);
  cv::line(image, b, d, color, 1, CV_AA, shift);
  
  
  
}

void polylines(cv::Mat image,
               const zarray_t* cpoints,
               const cv::Scalar& color,
               bool closed,
               int u) {

  std::vector<cv::Point> points;
  
  contour_to_points(cpoints, points, u);
  
  //contour_to_points(ci->points, points);
  const cv::Point* p0 = &(points[0]);
  int npts = points.size();
    
  cv::polylines(image, &p0, &npts, 1, closed, color, 1);

  if (u>1) {
    for (size_t i=0; i<points.size(); ++i) {
      cv::circle(image, points[i], u/4+1, color, -1);
    }
  }

  
}



/*
void homographyToPoseCV(at::real fx, at::real fy,
                        at::real tagSize,
                        const at::Mat& horig,
                        cv::Mat& rvec,
                        cv::Mat& tvec) {

  at::Point3 opoints[4] = {
    at::Point3(-1, -1, 0),
    at::Point3( 1, -1, 0),
    at::Point3( 1,  1, 0),
    at::Point3(-1,  1, 0)
  };

  at::Point ipoints[4];

  at::real s = 0.5*tagSize;

  for (int i=0; i<4; ++i) {
    ipoints[i] = project(horig, opoints[i].x, opoints[i].y);
    opoints[i] *= s;
  }

  at::real Kdata[9] = { 
    fx, 0, 0,
    0, fy, 0, 
    0, 0, 1
  };

  at::Mat Kmat(3, 3, Kdata);

  at::Mat dcoeffs = at::Mat::zeros(4, 1);

  cv::Mat_<at::Point3> omat(4, 1, opoints);
  cv::Mat_<at::Point> imat(4, 1, ipoints);

  cv::Mat r, t;

  cv::solvePnP(omat, imat, Kmat, dcoeffs, r, t);

  if (rvec.type() == CV_32F) {
    r.convertTo(rvec, rvec.type());
  } else {
    rvec = r;
  }

  if (tvec.type() == CV_32F) {
    t.convertTo(tvec, tvec.type());
  } else {
    tvec = t;
  }

}
*/
