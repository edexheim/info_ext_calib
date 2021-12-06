#include "calibration/backend/BackendParams.h"
#include <opencv2/core/persistence.hpp>

BackendParams::BackendParams(const std::string& path) {
  loadYaml(path);
}

BackendParams::~BackendParams() {

}

void BackendParams::loadYaml(const std::string& path) {

  // For some reason linker issue when using string directly
  cv::FileStorage fs(path.c_str(), cv::FileStorage::READ);

  // Segment accumulator
  int segment_size;
  fs["segment_size"] >> segment_size;
  seg_accumulator_.segment_size_ = static_cast<uint32_t>(segment_size);

  // Segment database
  int max_segments;
  fs["max_segments"] >> max_segments;
  seg_database_.max_segments_ = static_cast<uint32_t>(max_segments);
  int landmark_overlap_thresh;
  fs["landmark_overlap_thresh"] >> landmark_overlap_thresh;
  seg_database_.landmark_overlap_thresh_ = static_cast<size_t>(landmark_overlap_thresh);
  fs["use_incremental"] >> seg_database_.incremental_;
  fs["database_entropy_thresh"] >> seg_database_.database_entropy_thresh_;

  // Noise model
  fs["reproj_sigma"] >> noise_model_.reproj_sigma_;

  fs["stereo_use_epipolar"] >> noise_model_.stereo_use_epipolar_;
  fs["epipolar_use_sampson"] >> noise_model_.epipolar_use_sampson_;
  fs["epipolar_sigma"] >> noise_model_.epipolar_sigma_;

  fs["odom_sigma"] >> noise_model_.odom_sigma_;
  fs["pose_prior_sigma"] >> noise_model_.pose_prior_sigma_;
  fs["extrinsic_translation_sigma"] >> noise_model_.extrinsic_translation_sigma_;

  // Marginal covariance normalization
  fs["marg_cov_rotation_sigma"] >> marginal_covariance_.rotation_sigma_;
  fs["marg_cov_translation_sigma"] >> marginal_covariance_.translation_sigma_;

  // Relinerization
  std::vector<double> relin_ext;
  std::vector<double> relin_pose;
  double relin_landmark;
  fs["relin_extrinsics"] >> relin_ext;
  fs["relin_poses"] >> relin_pose;
  fs["relin_landmarks"] >> relin_landmark;
  // Extrinsics calibration
  relin_thresholds_['c'] = (gtsam::Vector(6) << 
      relin_ext[0], relin_ext[0], relin_ext[0],
      relin_ext[1], relin_ext[1], relin_ext[1]).finished();
  // Vehicle poses
  relin_thresholds_['x'] = (gtsam::Vector(6) << 
      relin_pose[0], relin_pose[0], relin_pose[0],
      relin_pose[1], relin_pose[1], relin_pose[1]).finished();
  // Landmarks
  relin_thresholds_['l'] = (gtsam::Vector(3) << 
      relin_landmark, relin_landmark, relin_landmark).finished();
  // std::cout << relin_thresholds_['c'] << std::endl;
  // std::cout << relin_thresholds_['x'] << std::endl;
  // std::cout << relin_thresholds_['l'] << std::endl;

  // Triangulation
  fs["rank_tol"] >> triangulation_.rankTolerance;
  fs["enable_EPI"] >> triangulation_.enableEPI;
  fs["landmark_dist_thresh"] >> triangulation_.landmarkDistanceThreshold;
  fs["max_reproj_thresh"] >> triangulation_.dynamicOutlierRejectionThreshold;

  fs.release();
}