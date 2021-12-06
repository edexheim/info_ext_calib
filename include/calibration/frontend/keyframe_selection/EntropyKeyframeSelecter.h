#pragma once

#include "calibration/frontend/keyframe_selection/KeyframeSelecter.h"
#include <unordered_map>


class EntropyKeyframeSelecter : public KeyframeSelecter {

  public:

    EntropyKeyframeSelecter(const FrontendParams::KeyframeSelecterParams& params);
    ~EntropyKeyframeSelecter();

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

    bool processFrame(
          uint32_t curr_frame_id,
          const Calibrations& calib,
          const gtsam::Values& active_values,
          const gtsam::Values& inactive_values,
          const FeatureRefs& active_refs,
          const FeatureRefs& inactive_refs,
          const Features& active_features,
          const Features& inactive_features);

    GenEpiFactors constructGenEpiFactors(
                    uint32_t curr_frame_id,
                    const FeatureRefs& refs,
                    const Features& features,
                    const Calibrations& calib);

    void getExtrinsicEntropy(
          const gtsam::Values& values,
          const std::vector<boost::shared_ptr<gtsam::GenEpiFactor> >& epi_factors,
          Eigen::Matrix3d& H_R,
          Eigen::Matrix3d& H_t);
    
    bool checkEntropy(double E_diff);


    gtsam::noiseModel::Base::shared_ptr meas_noise_;

    std::unordered_map<size_t, gtsam::Pose3> T_wv_hist_;

    std::vector<gtsam::Pose3> T_wv_buf_;
    std::vector<uint32_t> frame_id_buf_;

};