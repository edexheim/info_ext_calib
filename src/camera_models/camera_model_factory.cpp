#include "calibration/camera_models/camera_model_factory.h"

#include <boost/make_shared.hpp>
#include "calibration/camera_models/Pinhole.h"
#include "calibration/camera_models/PinholeRadtan.h"
#include "calibration/camera_models/PinholeKB.h"

#include <gtsam_unstable/slam/ProjectionFactorPPP.h>


gtsam::PinholeModel::shared_ptr CameraModelFactory::getCameraModel(
    CameraModelType camera_type,
    const gtsam::Vector& v, 
    const FrontendParams::UndistortParams& undistort_params) {

  gtsam::PinholeModel::shared_ptr model;

  switch (camera_type) {
    case CameraModelType::Pinhole:
      model = boost::make_shared<gtsam::Pinhole>(v);
      break;
    case CameraModelType::PinholeRadtan:
      model = boost::make_shared<gtsam::PinholeRadtan>(v, undistort_params);
      break;
    case CameraModelType::PinholeKB:
      model = boost::make_shared<gtsam::PinholeKB>(v, undistort_params);
      break;
  }

  return model;
}

// gtsam::NonlinearFactor::shared_ptr CameraModelFactory::getProjectionFactor(
//     const gtsam::PinholeModel::shared_ptr& pinhole_model,
//     const gtsam::Point2& measured, const gtsam::SharedNoiseModel& noise_model,
//     gtsam::Key poseKey, gtsam::Key transformKey,  gtsam::Key pointKey) {

//   gtsam::NonlinearFactor::shared_ptr factor;

//   switch (pinhole_model->getCameraModelType()) {
//     case Pinhole:
//       factor = boost::make_shared<gtsam::ProjectionFactorPPP<
//           gtsam::Pose3, 
//           gtsam::Point3, 
//           gtsam::Pinhole> >(
//               measured, noise_model, 
//               poseKey, transformKey, pointKey, 
//               (boost::dynamic_pointer_cast<gtsam::Pinhole>(pinhole_model))
//           );
//       break;
//     case PinholeRadtan:
//       factor = boost::make_shared<gtsam::ProjectionFactorPPP<
//           gtsam::Pose3, 
//           gtsam::Point3, 
//           gtsam::PinholeRadtan> >(
//               measured, noise_model, 
//               poseKey, transformKey, pointKey, 
//               (boost::dynamic_pointer_cast<gtsam::PinholeRadtan>(pinhole_model))
//           );
//       break;
//     case PinholeKB:
//       factor = factor = boost::make_shared<gtsam::ProjectionFactorPPP<
//           gtsam::Pose3, 
//           gtsam::Point3, 
//           gtsam::PinholeKB> >(
//               measured, noise_model, 
//               poseKey, transformKey, pointKey, 
//               (boost::dynamic_pointer_cast<gtsam::PinholeKB>(pinhole_model))
//           );
//       break;
//   }

//   return factor;
// }