#include "calibration/frontend/OutlierRejecter.h"
#include <opencv2/calib3d.hpp>
#include <gtsam/geometry/Point2.h>

std::vector<uchar> OutlierRejecter::getInliers(const std::vector<cv::Point2f>& p1,
                                                const std::vector<cv::Point2f>& p2,
                                                const gtsam::PinholeModel* calib) const {

  std::vector<uchar> status;

  // Convert points to Eigen

  if (params_.model_ == FrontendParams::OutRejParams::SevenPoint) {
    // TODO: Only undistort, do not create ray
    throw std::runtime_error("Seven Point not implented currently");
    // std::vector<cv::Point2f> p1_u = cam_.undistort(p1, true);
    // std::vector<cv::Point2f> p2_u = cam_.undistort(p2, true);

    // cv::findFundamentalMat( p1_u, p2_u, 
    //                         cv::FM_RANSAC, 
    //                         params_.ransac_thresh_, 
    //                         params_.ransac_prob_, 
    //                         status);
  }
  else if (params_.model_ == FrontendParams::OutRejParams::FivePoint) {

    std::vector<cv::Point2f> p1_u;
    p1_u.reserve(p1.size());
    for (int i=0; i<p1.size(); i++) {
      gtsam::Point2 p_u = calib->calibrate(Eigen::Vector2d(p1[i].x, p1[i].y));
      p1_u.push_back(cv::Point2f(p_u.x(), p_u.y()));
    }
    std::vector<cv::Point2f> p2_u;
    p2_u.reserve(p2.size());
    for (int i=0; i<p2.size(); i++) {
      gtsam::Point2 p_u = calib->calibrate(Eigen::Vector2d(p2[i].x, p2[i].y));
      p2_u.push_back(cv::Point2f(p_u.x(), p_u.y()));
    }

    if (p1_u.size() > 5 && p2_u.size() > 5) {

      cv::findEssentialMat( p1_u, p2_u, cv::Mat::eye(3,3,CV_32F),
                            cv::RANSAC,  
                            params_.ransac_prob_, 
                            params_.ransac_thresh_,
                            status);
    }
    else {
      status.resize(p1.size(), false);
    }
  }

  return status;
}