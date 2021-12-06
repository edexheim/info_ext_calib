#include "calibration/frontend/Camera.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

// debugging
#include <iostream>

Camera::Camera(const cv::Mat& K, const cv::Mat& distortion, bool is_fisheye)
  : K_(K), distortion_(distortion), is_fisheye_(is_fisheye) {

}

Camera::~Camera() {

}

std::vector<cv::Point2f> Camera::undistort(
  const std::vector<cv::Point2f>& points, bool reproject) const {

  std::vector<cv::Point2f> points_out;
  cv::InputArray R = cv::noArray();
  cv::Mat K;
  if (reproject) {
    K = K_;
  }

  if (!is_fisheye_) {
    cv::undistortPoints(points, points_out, K_, distortion_, R, K);
  }
  else {
    cv::fisheye::undistortPoints(points, points_out, K_, distortion_, R, K);
  }
  return points_out;
}

std::vector<Eigen::Vector3d> Camera::getUnitRays(
  const std::vector<cv::Point2f>& points) const {

  std::vector<Eigen::Vector3d> unit_rays;
  if (points.size() > 0) {
    std::vector<cv::Point2f> rays;
    if (!is_fisheye_) {
      cv::undistortPoints(points, rays, K_, distortion_);
    }
    else {
      cv::fisheye::undistortPoints(points, rays, K_, distortion_);
    }

    unit_rays.reserve(rays.size());
    for (const auto& r : rays) {
      unit_rays.push_back(Eigen::Vector3d(r.x, r.y, 1.0).normalized());
    }
  }

  return unit_rays;
}