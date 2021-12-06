#include "calibration/frontend/keyframe_selection/EntropyKeyframeSelecter.h"
#include "calibration/utils/gtsam_utils.h"

EntropyKeyframeSelecter::EntropyKeyframeSelecter(const FrontendParams::KeyframeSelecterParams& params)
  : KeyframeSelecter(params),
    // TODO: Should this be a parameter? And allow for robust?
    meas_noise_(gtsam::noiseModel::Isotropic::Precision(1, 1.0)) {

};

EntropyKeyframeSelecter::~EntropyKeyframeSelecter() {

};

bool EntropyKeyframeSelecter::checkNewKeyframe(
                      uint32_t curr_frame_id,
                      const gtsam::Pose3& T_wv_est,
                      const KeyframeSelecter::Calibrations& calib,
                      const KeyframeSelecter::Extrinsics& extrinsics,
                      const KeyframeSelecter::FeatureRefs& active_refs,
                      const KeyframeSelecter::Features& active_features,
                      const KeyframeSelecter::FeatureRefs& inactive_refs,
                      const KeyframeSelecter::Features& inactive_features) {

  bool new_kf = false;
  // New keyframe to start session
  if (T_wv_hist_.size() == 0) {
    new_kf = true;
  }
  else {
    // Add current pose to buffer for entropy processing
    T_wv_buf_.push_back(T_wv_est);
    frame_id_buf_.push_back(curr_frame_id);

    // TODO: These values should be synced with optimization?
    // Add active pose values
    gtsam::Values active_values;
    for (int l=0; l<active_refs.size(); l++) {
      for (int i=0; i<active_refs[l].size(); i++) {
        size_t frame_id = active_refs[l][i].frame_id_;
        // Convert from SE(3) to SO(3) x R^3 for epipolar constraints
        const gtsam::Pose3& Twv = T_wv_hist_.at(frame_id); 
        active_values.tryInsert(RwvKey(frame_id), 
          static_cast<const gtsam::Value&>(gtsam::GenericValue<gtsam::Rot3>(Twv.rotation())));
        active_values.tryInsert(twvKey(frame_id), 
          static_cast<const gtsam::Value&>(gtsam::GenericValue<gtsam::Point3>(Twv.translation())));
      }
    }
    active_values.insert(RwvKey(curr_frame_id), gtsam::Rot3(T_wv_est.rotation()) );
    active_values.insert(twvKey(curr_frame_id), gtsam::Point3(T_wv_est.translation()) );
    // Add inactive pose values
    gtsam::Values inactive_values;
    for (int i=0; i<T_wv_buf_.size(); i++) {
      // TODO: Check this indexing
      uint32_t frame_id = frame_id_buf_[i];
      // int frame_id = curr_frame_id - ( (T_wv_buf_.size()-1) - i);
      inactive_values.insert( RwvKey(frame_id), gtsam::Rot3(T_wv_buf_[i].rotation()) );
      inactive_values.insert( twvKey(frame_id), gtsam::Point3(T_wv_buf_[i].translation()) );
    }
    // Add current calibration values
    for (int l=0; l<active_refs.size(); l++) {
      // Convert from SE(3) to SO(3) x R^3 for epipolar constraints
      const gtsam::Pose3& Tvcl = extrinsics[l]; 
      active_values.insert(RvcKey(l), Tvcl.rotation());
      active_values.insert(tvcKey(l), Tvcl.translation());
      inactive_values.insert(RvcKey(l), Tvcl.rotation());
      inactive_values.insert(tvcKey(l), Tvcl.translation());
    }
    // Check if new keyframe
    new_kf = this->processFrame(
                    curr_frame_id, calib, 
                    active_values, inactive_values, 
                    active_refs, inactive_refs,
                    active_features, inactive_features);
  }
  
  if (new_kf) {
     // Reset pose buffer - 0th index is keyframe
    T_wv_buf_.clear();
    T_wv_buf_.push_back(T_wv_est);
    frame_id_buf_.clear();
    frame_id_buf_.push_back(curr_frame_id);
  }

  T_wv_hist_.insert({curr_frame_id, T_wv_est});

  return new_kf;

}

bool EntropyKeyframeSelecter::processFrame(
                        uint32_t curr_frame_id,
                        const KeyframeSelecter::Calibrations& calib,
                        const gtsam::Values& active_values,
                        const gtsam::Values& inactive_values,
                        const KeyframeSelecter::FeatureRefs& active_refs,
                        const KeyframeSelecter::FeatureRefs& inactive_refs,
                        const KeyframeSelecter::Features& active_features,
                        const KeyframeSelecter::Features& inactive_features) {

  uint8_t num_cam = active_refs.size();

  // Active information calculation
  GenEpiFactors active_factors 
    = this->constructGenEpiFactors(curr_frame_id, active_refs, active_features, calib);

  std::vector<Eigen::Matrix3d> H_R_active(num_cam, Eigen::Matrix3d::Zero());
  std::vector<Eigen::Matrix3d> H_t_active(num_cam, Eigen::Matrix3d::Zero());
  for (int l=0; l<num_cam; l++) {
    this->getExtrinsicEntropy(active_values, active_factors[l], 
                              H_R_active[l], H_t_active[l]);
  }

  // Inactive information calculation
  GenEpiFactors inactive_factors
    = this->constructGenEpiFactors(curr_frame_id, inactive_refs, inactive_features, calib);

  std::vector<Eigen::Matrix3d> H_R_inactive(num_cam, Eigen::Matrix3d::Zero());
  std::vector<Eigen::Matrix3d> H_t_inactive(num_cam, Eigen::Matrix3d::Zero());
  for (int l=0; l<num_cam; l++) {
    this->getExtrinsicEntropy(inactive_values, inactive_factors[l], 
                              H_R_inactive[l], H_t_inactive[l]);
  }

  // Get negative entropy across all extrinsics
  // TODO: keep as single metric or per extrinsic block?
  double E_active = 0;
  double E_total = 0;
  for (int l=0; l<num_cam; l++) {
    E_active += std::log(H_R_active[l].determinant());
    E_active += std::log(H_t_active[l].determinant());
    E_total += std::log((H_R_active[l] + H_R_inactive[l]).determinant());
    E_total += std::log((H_t_active[l] + H_t_inactive[l]).determinant());
  }
  E_active = 0.5*E_active - (6*num_cam)*(1 + 2*3.14159)/2.0;
  E_total = 0.5*E_total - (6*num_cam)*(1 + 2*3.14159)/2.0;

  bool new_kf;
  if (!std::isfinite(E_total)) {
    new_kf = true;
  }
  else {
    new_kf = this->checkEntropy(E_active - E_total);
  }

  // std::cout << E_active << " " << E_total << " " << static_cast<int>(new_kf) << std::endl;

  return new_kf;
}

KeyframeSelecter::GenEpiFactors EntropyKeyframeSelecter::constructGenEpiFactors(
                                      uint32_t curr_frame_id,
                                      const FeatureRefs& refs,
                                      const Features& features,
                                      const Calibrations& calib) {
                                        
  uint8_t num_cam = refs.size();
  GenEpiFactors epi_factors(num_cam);
  for (int l=0; l<num_cam; l++) {
    int num_matches = refs[l].size();
    for (int i=0; i<num_matches; i++) {
      // Convert to ray format
      gtsam::Point2 p_u_ref = calib[l]->calibrate(Eigen::Vector2d(refs[l][i].pt_.x, refs[l][i].pt_.y));
      gtsam::Point3 ref_ray = gtsam::Point3(p_u_ref.x(), p_u_ref.y(), 1.0).normalized();
      gtsam::Point2 p_u_curr = calib[l]->calibrate(Eigen::Vector2d(features[l][i].x, features[l][i].y));
      gtsam::Point3 curr_ray = gtsam::Point3(p_u_curr.x(), p_u_curr.y(), 1.0).normalized();
      // Create factor
      boost::shared_ptr<gtsam::GenEpiFactor> factor = 
        boost::make_shared<gtsam::GenEpiFactor>(  
        RwvKey(refs[l][i].frame_id_), twvKey(refs[l][i].frame_id_),
        RwvKey(curr_frame_id), twvKey(curr_frame_id),
        RvcKey(l), tvcKey(l),
        ref_ray, curr_ray,
        meas_noise_);

      epi_factors[l].push_back(factor);
    }
  }
  return epi_factors;
}

void EntropyKeyframeSelecter::getExtrinsicEntropy(const gtsam::Values& values,
                                  const std::vector<boost::shared_ptr<gtsam::GenEpiFactor> >& epi_factors,
                                  Eigen::Matrix3d& H_R,
                                  Eigen::Matrix3d& H_t) {

  H_R = Eigen::Matrix3d::Zero();
  H_t = Eigen::Matrix3d::Zero(); 

  for (int i=0; i<epi_factors.size(); i++) {
    gtsam::Matrix J_Rvc, J_tvc;
    epi_factors[i]->evaluateError(values.at<gtsam::Rot3>(epi_factors[i]->key1()), 
                                  values.at<gtsam::Point3>(epi_factors[i]->key2()), 
                                  values.at<gtsam::Rot3>(epi_factors[i]->key3()), 
                                  values.at<gtsam::Point3>(epi_factors[i]->key4()), 
                                  values.at<gtsam::Rot3>(epi_factors[i]->key5()), 
                                  values.at<gtsam::Point3>(epi_factors[i]->key6()), 
                                  boost::none, boost::none, boost::none, boost::none, 
                                  J_Rvc, J_tvc);
    
    H_R += J_Rvc.transpose() * J_Rvc;
    H_t += J_tvc.transpose() * J_tvc; 
  }

}

bool EntropyKeyframeSelecter::checkEntropy(double E_diff) {
  // E_diff should be negative since active-total
  bool new_kf = false;
  if (E_diff <= std::log(params_.keyframe_prob_)) {
    new_kf = true;
  }
  return new_kf;
}