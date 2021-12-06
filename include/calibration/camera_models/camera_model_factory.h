#pragma once

#include "calibration/utils/basic_types.h"

#include "calibration/camera_models/PinholeModel.h"
#include <gtsam/base/Vector.h>
#include "calibration/frontend/FrontendParams.h"
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Point2.h>

class CameraModelFactory {
  public:
    static gtsam::PinholeModel::shared_ptr getCameraModel(
        CameraModelType camera_type,
        const gtsam::Vector& v, 
        const FrontendParams::UndistortParams& undistort_params);

    // static gtsam::NonlinearFactor::shared_ptr getProjectionFactor(
    //     const gtsam::PinholeModel::shared_ptr& pinhole_model,
    //     const gtsam::Point2& measured, const gtsam::SharedNoiseModel& noise_model,
    //     gtsam::Key poseKey, gtsam::Key transformKey,  gtsam::Key pointKey);
};