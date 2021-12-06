#pragma once

#include "calibration/frontend/keyframe_selection/KeyframeSelecter.h"
#include <unordered_map>


class FeatureKeyframeSelecter : public KeyframeSelecter {

  public:

    FeatureKeyframeSelecter(const FrontendParams::KeyframeSelecterParams& params);
    ~FeatureKeyframeSelecter();

    virtual bool checkNewKeyframe(
          uint32_t curr_frame_id,
          const gtsam::Pose3& T_wv_est,
          const Calibrations& calib,
          const Extrinsics& extrinsics,
          const FeatureRefs& active_refs,
          const Features& active_features,
          const FeatureRefs& inactive_refs,
          const Features& inactive_features);

  private:
};