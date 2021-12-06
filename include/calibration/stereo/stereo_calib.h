#pragma once

#include <vector>
#include "calibration/camera_models/PinholeModel.h"
#include <gtsam/geometry/Pose3.h>
#include "calibration/frontend/Frame.h"
#include "calibration/stereo/stereo_params.h"
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/geometry/triangulation.h>
#include "calibration/utils/basic_types.h"

namespace stereo {

void setupCalib(
  const std::vector<gtsam::PinholeModel::shared_ptr>& intrinsics,
  const std::vector<gtsam::Pose3>& extrinsics_estimate,
  const std::vector<std::shared_ptr<Frame> >& frames,
  std::vector<std::pair<size_t, size_t> >& landmark_landmark_matches,
  std::vector<std::pair<size_t, StereoMatch> >& landmark_point_matches,
  std::vector<std::pair<StereoMatch, StereoMatch> >& point_point_matches,
  const Params& stereo_params);

} // end namespace stereo