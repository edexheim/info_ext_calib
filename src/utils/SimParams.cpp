#include "calibration/utils/SimParams.h"
#include <opencv2/core/persistence.hpp>

SimParams::SimParams(const std::string& path) {
  loadYaml(path);
}

SimParams::~SimParams() {

}

void SimParams::loadYaml(const std::string& path) {

  // For some reason linker issue when using string directly
  cv::FileStorage fs(path.c_str(), cv::FileStorage::READ);

  // Odometry
  std::vector<double> odom_sigmas;
  fs["odom_sigmas"] >> odom_sigmas;
  odom_noise_model_= gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector(6) <<
        gtsam::Vector3::Constant(odom_sigmas[0]), 
        gtsam::Vector3::Constant(odom_sigmas[1])).finished());
  
  int odom_seed;
  fs["odom_seed"] >> odom_seed;
  odom_seed_ = static_cast<uint64_t>(odom_seed);

  // Extrinsics
  std::vector<double> extrinsics_sigmas;
  fs["extrinsics_sigmas"] >> extrinsics_sigmas;
  extrinsics_noise_model_= gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector(6) <<
        gtsam::Vector3::Constant(extrinsics_sigmas[0]), 
        gtsam::Vector3::Constant(extrinsics_sigmas[1])).finished());

  int extrinsics_seed;
  fs["extrinsics_seed"] >> extrinsics_seed;
  extrinsics_seed_ = static_cast<uint64_t>(extrinsics_seed);

  fs.release();
}