#ifndef BASIC_TYPES_H
#define BASIC_TYPES_H

#include <map>
#include "calibration/frontend/Frame.h"
#include <opencv2/core/types.hpp>
#include <gtsam/geometry/Pose3.h>
#include "calibration/factors/projection/ExtrinsicsProjectionFactor.h"

typedef double Timestamp;

struct MultiFrameInput {
  typedef std::shared_ptr<MultiFrameInput> shared_ptr;

  size_t frame_id_;
  Timestamp timestamp_;

  // Image data
  std::vector<std::shared_ptr<Frame> > frames_;
  std::vector<cv::Mat> masks_;

  // Initial estimates
  gtsam::Pose3 T_wv_est_;
  std::vector<gtsam::PinholeModel::shared_ptr> intrinsics_;
  std::vector<gtsam::Pose3> extrinsics_;
};

struct KeyframeInput {
  typedef std::shared_ptr<KeyframeInput> shared_ptr;

  size_t frame_id_;

  // Images needed for stereo matching, monocular features for backend
  std::vector<std::shared_ptr<Frame> > frames_;

  // Initial estimates
  gtsam::Pose3 T_wv_est_;
  std::vector<gtsam::PinholeModel::shared_ptr> intrinsics_;
  std::vector<gtsam::Pose3> extrinsics_;
};

struct BackendOutput {
  typedef std::shared_ptr<BackendOutput> shared_ptr;

  size_t frame_id_;
  std::vector<gtsam::Pose3> extrinsics_;
  std::vector<gtsam::Matrix6> marginal_information_;

  std::vector<std::vector<gtsam::Pose3> > segment_poses_;
  std::vector<gtsam::Point3> tracked_landmarks_;
  std::vector<gtsam::Point3> stereo_landmarks_;
  
  std::vector<std::vector<double> > segment_info_content_;

  std::vector<double> database_info_content_;
};

struct FeatureReference {
  FeatureReference() {};

  FeatureReference(size_t frame_id, const cv::Point2f& pt, size_t feat_id)
    : frame_id_(frame_id), pt_(pt), feat_id_(feat_id)
  {};

  size_t frame_id_;
  cv::Point2f pt_;
  size_t feat_id_;
};

struct StereoMatch {
  StereoMatch(const cv::Point2f pt, size_t cam_id)
      : pt_(pt), cam_id_(cam_id) {}

  cv::Point2f pt_;
  size_t cam_id_;
};

typedef gtsam::ExtrinsicsProjectionFactor<gtsam::Pose3, gtsam::Point3> 
    ProjectionFactor;

typedef std::vector<ProjectionFactor::shared_ptr> ProjectionFactorBundle;

typedef std::map<size_t, ProjectionFactorBundle> CameraObservations;

typedef std::vector<std::pair<size_t, size_t> > FeatureIdMatches;

#endif