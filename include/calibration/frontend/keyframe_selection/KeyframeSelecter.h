#pragma once

#include <cstdint>
#include <Eigen/Dense>
#include <gtsam/geometry/Pose3.h>
#include "gtsam/nonlinear/Values.h"
#include "calibration/factors/epipolar/GenEpiFactor.h"
#include "calibration/utils/basic_types.h"
#include "calibration/camera_models/PinholeModel.h"
#include "calibration/frontend/FrontendParams.h"
#include <unordered_map>


class KeyframeSelecter {

  protected:

    typedef std::vector<std::vector<boost::shared_ptr<gtsam::GenEpiFactor> > > GenEpiFactors;
    typedef std::vector<std::vector<FeatureReference> > FeatureRefs;
    typedef std::vector<std::vector<cv::Point2f> > Features;
    typedef std::vector<gtsam::PinholeModel::shared_ptr> Calibrations;
    typedef std::vector<gtsam::Pose3> Extrinsics;

  public:

    KeyframeSelecter(const FrontendParams::KeyframeSelecterParams& params);
    virtual ~KeyframeSelecter();

    virtual bool checkNewKeyframe(
          uint32_t curr_frame_id,
          const gtsam::Pose3& T_wv_est,
          const Calibrations& calib,
          const Extrinsics& extrinsics,
          const FeatureRefs& active_refs,
          const Features& active_features,
          const FeatureRefs& inactive_refs,
          const Features& inactive_features) = 0;

  protected:

    const FrontendParams::KeyframeSelecterParams params_; 
};