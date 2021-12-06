#include "calibration/frontend/keyframe_selection/FeatureKeyframeSelecter.h"
#include "calibration/utils/gtsam_utils.h"

FeatureKeyframeSelecter::FeatureKeyframeSelecter(const FrontendParams::KeyframeSelecterParams& params)
  : KeyframeSelecter(params)
{

};

FeatureKeyframeSelecter::~FeatureKeyframeSelecter() {

};

bool FeatureKeyframeSelecter::checkNewKeyframe(
                      uint32_t curr_frame_id,
                      const gtsam::Pose3& T_wv_est,
                      const KeyframeSelecter::Calibrations& calib,
                      const KeyframeSelecter::Extrinsics& extrinsics,
                      const KeyframeSelecter::FeatureRefs& active_refs,
                      const KeyframeSelecter::Features& active_features,
                      const KeyframeSelecter::FeatureRefs& inactive_refs,
                      const KeyframeSelecter::Features& inactive_features) {

  size_t num_active_features_active = 0;
  for (const auto& cam_refs : active_refs)
    num_active_features_active += cam_refs.size();

  size_t num_active_features_inactive = 0;
  for (const auto& cam_refs : inactive_refs)
    num_active_features_inactive += cam_refs.size();

  size_t num_active_features_total = num_active_features_inactive + num_active_features_active;

  double feature_ratio = 
      (static_cast<double>(num_active_features_active))
      / (static_cast<double>(num_active_features_total));
  
  bool new_kf = false;
  if (feature_ratio <= params_.keyframe_prob_)
    new_kf = true;
  
  return new_kf;
}