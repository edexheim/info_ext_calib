#ifndef SEGMENT_ACCUMULATOR_H
#define SEGMENT_ACCUMULATOR_H

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Ordering.h>
#include "calibration/backend/Segment.h"
#include "calibration/backend/BackendParams.h"
#include "calibration/camera_models/PinholeModel.h"
#include "calibration/utils/basic_types.h"


class SegmentAccumulator {

  public:

    SegmentAccumulator(
        const BackendParams::SegmentAccumulatorParams& params,
        const BackendParams::NoiseModelParams& noise_params);

    virtual ~SegmentAccumulator();

    void addVehicleEdge(int i, int j, const gtsam::Pose3& T_ij, double info);

    bool handleKeyframe(const KeyframeInput::shared_ptr& backend_input);

    void addLandmarkLandmarkMatches(const std::vector<std::pair<size_t, size_t> >& matches);
    void addLandmarkPointMatches(
        size_t frame_id,
        const std::vector<gtsam::PinholeModel::shared_ptr>& intrinsics,
        const std::vector<std::pair<size_t, StereoMatch> >& matches);
    void addPointPointMatches(
        size_t frame_id,
        const std::vector<gtsam::PinholeModel::shared_ptr>& intrinsics,
        const std::vector<std::pair<StereoMatch, StereoMatch> >& matches);

    Segment generateSegment();

    void reset();

  protected:

    // Parameters
    const BackendParams::SegmentAccumulatorParams params_;
    const BackendParams::NoiseModelParams noise_params_;

    // Session variables (persists for program runtime)
    size_t stereo_feature_id_;

    // Segment variables (cleared for each segment)
    gtsam::NonlinearFactorGraph seg_graph_odom_;
    CameraObservations seg_graph_obs_;
    CameraObservations stereo_obs_;
    FeatureIdMatches seg_feature_id_matches_;

    gtsam::Values seg_values_; 

    size_t num_poses_;
    bool new_segment_;
    size_t first_id_;

    // Prev keyframe variables
    int prev_kf_id_;
    gtsam::Pose3 T_wv_prev_;



};

#endif