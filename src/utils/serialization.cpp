#include "calibration/utils/serialization.h"
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Pose3.h>
#include "calibration/camera_models/PinholeModel.h"
#include "calibration/camera_models/Pinhole.h"
#include "calibration/camera_models/PinholeRadtan.h"
#include "calibration/camera_models/PinholeKB.h"
#include "calibration/factors/epipolar/EpipolarFactor.h"
#include "calibration/factors/epipolar/GenEpiFactor.h"
#include "calibration/factors/projection/ExtrinsicsProjectionFactor.h"
#include <gtsam/base/serialization.h>
#include <gtsam_unstable/slam/ProjectionFactorPPP.h>

// gtsam nonlinear factor graph boost serialization 
// noise models
BOOST_CLASS_EXPORT_GUID(gtsam::SharedNoiseModel, "gtsam_SharedNoiseModel");
BOOST_CLASS_EXPORT_GUID(gtsam::SharedDiagonal, "gtsam_SharedDiagonal");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Constrained, "gtsam_noiseModel_Constrained");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Diagonal, "gtsam_noiseModel_Diagonal");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Gaussian, "gtsam_noiseModel_Gaussian");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Unit, "gtsam_noiseModel_Unit");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Isotropic, "gtsam_noiseModel_Isotropic");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Robust, "gtsam_noiseModel_Robust");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::mEstimator::Huber, "gtsam_noiseModel_Huber");
// value types needed for factors
BOOST_CLASS_EXPORT(gtsam::Rot3);
BOOST_CLASS_EXPORT(gtsam::Point3);
BOOST_CLASS_EXPORT(gtsam::Pose3);
BOOST_CLASS_EXPORT(gtsam::PinholeModel);
BOOST_CLASS_EXPORT(gtsam::Pinhole);
BOOST_CLASS_EXPORT(gtsam::PinholeRadtan);
BOOST_CLASS_EXPORT(gtsam::PinholeKB);
// factors
BOOST_CLASS_EXPORT_GUID(gtsam::PriorFactor<gtsam::Pose3>, "gtsam::PriorFactorPose3");
BOOST_CLASS_EXPORT_GUID(gtsam::BetweenFactor<gtsam::Pose3>, "gtsam::BetweenFactorPose3");
BOOST_CLASS_EXPORT_GUID(gtsam::GenEpiFactor, "gtsam::GenEpiFactor");
BOOST_CLASS_EXPORT_GUID(gtsam::EpipolarFactor, "gtsam::EpipolarFactor");
typedef gtsam::ExtrinsicsProjectionFactor<gtsam::Pose3, gtsam::Point3> ExtrinsicsProjectionFactor_Pose3_Point3;
BOOST_CLASS_EXPORT_GUID(ExtrinsicsProjectionFactor_Pose3_Point3, "gtsam::ExtrinsicsProjectionFactor_Pose3_Point3");

// gtsam values serialization
GTSAM_VALUE_EXPORT(gtsam::Rot3);
GTSAM_VALUE_EXPORT(gtsam::Point3);
GTSAM_VALUE_EXPORT(gtsam::Pose3);
// GTSAM_VALUE_EXPORT(gtsam::PinholeModel);
GTSAM_VALUE_EXPORT(gtsam::Pinhole);
GTSAM_VALUE_EXPORT(gtsam::PinholeRadtan);
GTSAM_VALUE_EXPORT(gtsam::PinholeKB);

namespace io {

bool serializeGraph(const gtsam::NonlinearFactorGraph& graph, 
    const std::string& filename) {
  return gtsam::serializeToBinaryFile(graph, filename);
}

bool serializeValues(const gtsam::Values& values,
    const std::string& filename) {
  return gtsam::serializeToBinaryFile(values, filename);
}

bool serializeSegment(const Segment& segment,
    const std::string& filename) {
  return gtsam::serializeToBinaryFile(segment, filename);
}

bool deserializeSegment(const std::string& filename,
    Segment& segment) {
  return gtsam::deserializeFromBinaryFile(filename, segment);
}

bool serializeExtrinsics(const std::vector<gtsam::Pose3>& extrinsics,
    const std::string& filename) {

  return gtsam::serializeToBinaryFile(extrinsics, filename);
}

bool deserializeExtrinsics(const std::string& filename,
    std::vector<gtsam::Pose3>& extrinsics) {

  return gtsam::deserializeFromBinaryFile(filename, extrinsics);
}

} // end namespace io