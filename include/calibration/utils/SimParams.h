#ifndef SIM_PARAMS_H
#define SIM_PARAMS_H

#include <string>
#include <gtsam/linear/NoiseModel.h>

struct SimParams {

  SimParams(const std::string& path);
  ~SimParams();
  void loadYaml(const std::string& path);

  gtsam::noiseModel::Gaussian::shared_ptr odom_noise_model_;
  uint64_t odom_seed_;

  gtsam::noiseModel::Gaussian::shared_ptr extrinsics_noise_model_;
  uint64_t extrinsics_seed_;
    
};

#endif