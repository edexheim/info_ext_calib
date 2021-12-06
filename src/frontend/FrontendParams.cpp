#include "calibration/frontend/FrontendParams.h"
#include <opencv2/core/persistence.hpp>

FrontendParams::FrontendParams(const std::string& path) {
  loadYaml(path);
}

FrontendParams::~FrontendParams() {

}

void FrontendParams::loadYaml(const std::string& path) {

  // For some reason linker issue when using string directly
  cv::FileStorage fs(path.c_str(), cv::FileStorage::READ);

  // Detector
  std::vector<int> grid_size_vec;
  fs["grid_size"] >> grid_size_vec;
  detector_.grid_size_ = cv::Size(grid_size_vec[0], grid_size_vec[1]);
  fs["num_features_per_cell"] >> detector_.num_features_per_cell_;
  fs["min_dist"] >> detector_.min_dist_;

  int tracker_type;
  fs["tracker_type"] >> tracker_type;
  tracker_type_ = static_cast<FrontendParams::TrackerType>(tracker_type);

  // KLT Tracker
  fs["window_size"] >> tracker_.window_size_;
  fs["max_level"] >> tracker_.max_level_;
  fs["max_iter"] >> tracker_.max_iter_;
  fs["convergence_eps"] >> tracker_.convergence_eps_;
  fs["min_eig_thresh"] >> tracker_.min_eig_thresh_;
  fs["two_way_tracking"] >> tracker_.two_way_tracking_;
  fs["two_way_thresh"] >> tracker_.two_way_thresh_;
  fs["pattern"] >> tracker_.pattern_;

  // Outlier rejection
  fs["ransac_thresh"] >> out_rej_.ransac_thresh_;
  fs["ransac_prob"] >> out_rej_.ransac_prob_;
  int model;
  fs["model"] >> model;
  out_rej_.model_ = static_cast<FrontendParams::OutRejParams::Model>(model);

  // Keyframe selecter
  int kf_type;
  fs["keyframe_type"] >> kf_type;
  kf_seleceter_.keyframe_type_ = static_cast<FrontendParams::KeyframeSelecterParams::KfType>(kf_type);
  fs["keyframe_prob"] >> kf_seleceter_.keyframe_prob_;

  // Undistortion params
  fs["undistort_max_iter"] >> undistort_.max_iter_;
  fs["undistort_tol"] >> undistort_.tol_;
  fs["undistort_throw_err"] >> undistort_.throw_err_;

  fs.release();
}