#include "calibration/backend/SegmentAccumulator.h"
#include "calibration/utils/gtsam_utils.h"
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Pose3.h>
#include "calibration/utils/gtsam_utils.h"
#include "calibration/camera_models/camera_model_factory.h"

SegmentAccumulator::SegmentAccumulator(
  const BackendParams::SegmentAccumulatorParams& params,
  const BackendParams::NoiseModelParams& noise_params)
  : params_(params), noise_params_(noise_params),
    stereo_feature_id_(0),
    num_poses_(0), new_segment_(true), 
    prev_kf_id_(-1)  {

}

SegmentAccumulator::~SegmentAccumulator() {

}

void SegmentAccumulator::addVehicleEdge(
    int i, int j, const gtsam::Pose3& T_ij, double sigma) {

  auto noise_model = gtsam::noiseModel::Isotropic::Sigma(6, sigma);

  auto between_factor = boost::make_shared<gtsam::BetweenFactor<gtsam::Pose3> >
      (gtsam::BetweenFactor<gtsam::Pose3>(
          TwvKey(i), 
          TwvKey(j), 
          T_ij, noise_model) );

  seg_graph_odom_.add(between_factor);
}

bool SegmentAccumulator::handleKeyframe(
    const KeyframeInput::shared_ptr& backend_input) {

  size_t frame_id = backend_input->frame_id_;
  const gtsam::Pose3& T_wv_est = backend_input->T_wv_est_;

  // Start of session - initialize first pose from odometry
  if (prev_kf_id_ < 0) {
    // add phantom odometry for consistency
    seg_graph_odom_.add(gtsam::BetweenFactor<gtsam::Pose3>::shared_ptr());
  }
  else {
    // Get relative pose wrt last keyframe
    gtsam::Pose3 T_rel_key = T_wv_prev_.inverse() * T_wv_est;
    // Assuming constant odometry uncertainty, but interval between frames matters
    double sigma_squared = noise_params_.odom_sigma_ * noise_params_.odom_sigma_;
    double odom_sigma = std::sqrt( (frame_id - prev_kf_id_) * sigma_squared);
    this->addVehicleEdge(prev_kf_id_, frame_id, 
        T_rel_key, odom_sigma);
  }

  // Add projection factors
  auto gaussian_model = gtsam::noiseModel::Isotropic::Sigma(2, 
      noise_params_.reproj_sigma_);
  auto proj_noise_model = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Huber::Create(1.345), gaussian_model);

  for (int l = 0; l < backend_input->frames_.size(); l++) {
    for (int i = 0; i < backend_input->frames_[l]->features_.size(); i++) {
      size_t ft_id = backend_input->frames_[l]->feature_ids_[i];
      const cv::Point2f& pt = backend_input->frames_[l]->features_[i];
      gtsam::Point2 measured(pt.x, pt.y);

      // TODO: Factor factory
      seg_graph_obs_[ft_id].push_back(
          boost::make_shared<gtsam::ExtrinsicsProjectionFactor<gtsam::Pose3, gtsam::Point3>>(
              measured, proj_noise_model, 
              TwvKey(frame_id), 
              TvcKey(l), 
              lKey(ft_id), 
              backend_input->intrinsics_[l]) );
    }
  }

  // Set start of segment
  if (new_segment_) {
    new_segment_ = false;
    first_id_ = frame_id;
  }

  // Add new pose vars
  seg_values_.insert(TwvKey(frame_id), T_wv_est);

  prev_kf_id_ = frame_id;
  T_wv_prev_ = T_wv_est;

  num_poses_++;

  return num_poses_ == params_.segment_size_;
}

void SegmentAccumulator::addLandmarkLandmarkMatches(
    const FeatureIdMatches& matches) {

  seg_feature_id_matches_.insert(
      seg_feature_id_matches_.end(),
      matches.begin(),
      matches.end() );
}

void SegmentAccumulator::addLandmarkPointMatches(
    size_t frame_id,
    const std::vector<gtsam::PinholeModel::shared_ptr>& intrinsics,
    const std::vector<std::pair<size_t, StereoMatch> >& matches) {

  auto gaussian_model = gtsam::noiseModel::Isotropic::Sigma(2, 
      noise_params_.reproj_sigma_);
  auto proj_noise_model = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Huber::Create(1.345), gaussian_model);

  for (const auto& [ft_id, obs] : matches) {
    gtsam::Point2 measured(obs.pt_.x, obs.pt_.y);
    seg_graph_obs_[ft_id].push_back(
        boost::make_shared<gtsam::ExtrinsicsProjectionFactor<gtsam::Pose3, gtsam::Point3>>(
            measured, proj_noise_model, 
            TwvKey(frame_id), 
            TvcKey(obs.cam_id_), 
            lKey(ft_id), 
            intrinsics[obs.cam_id_]) );
  }
}

void SegmentAccumulator::addPointPointMatches(
    size_t frame_id,
    const std::vector<gtsam::PinholeModel::shared_ptr>& intrinsics,
    const std::vector<std::pair<StereoMatch, StereoMatch> >& matches) {


  auto gaussian_model = gtsam::noiseModel::Isotropic::Sigma(2, 
      noise_params_.reproj_sigma_);
  auto proj_noise_model = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Huber::Create(1.345), gaussian_model);

  for (const auto& [obs1, obs2] : matches) {
    gtsam::Point2 measured1(obs1.pt_.x, obs1.pt_.y);
    gtsam::Point2 measured2(obs2.pt_.x, obs2.pt_.y);

    stereo_obs_[stereo_feature_id_].push_back(
        boost::make_shared<gtsam::ExtrinsicsProjectionFactor<gtsam::Pose3, gtsam::Point3>>(
            measured1, proj_noise_model, 
            TwvKey(frame_id), 
            TvcKey(obs1.cam_id_), 
            lStereoKey(stereo_feature_id_), 
            intrinsics[obs1.cam_id_]) );
    
    stereo_obs_[stereo_feature_id_].push_back(
        boost::make_shared<gtsam::ExtrinsicsProjectionFactor<gtsam::Pose3, gtsam::Point3>>(
            measured2, proj_noise_model, 
            TwvKey(frame_id), 
            TvcKey(obs2.cam_id_), 
            lStereoKey(stereo_feature_id_), 
            intrinsics[obs2.cam_id_]) );

    stereo_feature_id_++;
  }
}

Segment SegmentAccumulator::generateSegment() {

  return Segment(seg_graph_odom_, seg_graph_obs_, stereo_obs_, seg_feature_id_matches_, 
      seg_values_, first_id_);
}

void SegmentAccumulator::reset() {
  new_segment_ = true;
  num_poses_ = 0;
  seg_graph_odom_.resize(0);
  seg_graph_obs_.clear();
  stereo_obs_.clear();
  seg_feature_id_matches_.clear();
  seg_values_.clear();
}