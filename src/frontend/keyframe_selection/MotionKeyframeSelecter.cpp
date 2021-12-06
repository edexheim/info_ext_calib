#include "calibration/frontend/keyframe_selection/MotionKeyframeSelecter.h"
#include "calibration/utils/gtsam_utils.h"

MotionKeyframeSelecter::MotionKeyframeSelecter(const FrontendParams::KeyframeSelecterParams& params)
  : KeyframeSelecter(params),
    first_(true),
    T_wv_prev_()
{

};

MotionKeyframeSelecter::~MotionKeyframeSelecter() {

};

bool MotionKeyframeSelecter::checkNewKeyframe(
                      uint32_t curr_frame_id,
                      const gtsam::Pose3& T_wv_est,
                      const KeyframeSelecter::Calibrations& calib,
                      const KeyframeSelecter::Extrinsics& extrinsics,
                      const KeyframeSelecter::FeatureRefs& active_refs,
                      const KeyframeSelecter::Features& active_features,
                      const KeyframeSelecter::FeatureRefs& inactive_refs,
                      const KeyframeSelecter::Features& inactive_features) {

  bool new_kf = false;
  if (first_) {
    new_kf = true;
    first_ = false;
    T_wv_prev_ = T_wv_est;
  }
  else {
    if (T_wv_prev_.range(T_wv_est) > params_.keyframe_prob_) {
      new_kf = true;
      T_wv_prev_ = T_wv_est;
    }
  }
  
  return new_kf;
}