#include "calibration/frontend/FeatureMatcher.h"

FeatureMatcher::FeatureMatcher() {

}

FeatureMatcher::~FeatureMatcher() {

}

void FeatureMatcher::initialize() {
  // TODO: user input parameters and type of matcher
  int norm_type = cv::NORM_HAMMING; // for binary features
  bool consistency_check = false; // checks if matches consistent both ways
  dist_thresh_ = 10.0;
  matcher_ = cv::BFMatcher::create(norm_type, consistency_check);
}

std::vector<cv::DMatch> FeatureMatcher::match( 
    const std::vector<cv::KeyPoint>& kp1, const std::vector<cv::KeyPoint>& kp2,
    const cv::Mat& desc1, const cv::Mat& desc2,
    gtsam::Pose3::shared_ptr ext1, gtsam::Pose3::shared_ptr ext2) {

  float dist_sq_thresh = dist_thresh_ * dist_thresh_;

  // Filter out matches using epipolar line distance
  cv::Mat mask(prev_desc.rows, curr_desc.rows, CV_8UC1);
  

  std::vector<cv::DMatch> matches;
  matcher_->match(prev_desc, curr_desc, matches, mask);

  return matches;
}