#ifndef CAMERA_H
#define CAMERA_H

#include <opencv2/core/mat.hpp>
#include <Eigen/Dense>

class Camera {

  public:

    Camera(const cv::Mat& K, const cv::Mat& distortion, bool fisheye);
    ~Camera();

    std::vector<cv::Point2f> undistort(
      const std::vector<cv::Point2f>& points, bool reproject) const;
    std::vector<Eigen::Vector3d> getUnitRays(
      const std::vector<cv::Point2f>& points) const;

  protected:
    cv::Mat K_;
    cv::Mat distortion_;
    bool is_fisheye_;

};

#endif