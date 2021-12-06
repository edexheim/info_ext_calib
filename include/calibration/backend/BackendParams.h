#ifndef BACKEND_PARAMS_H
#define BACKEND_PARAMS_H

#include <string>
#include <gtsam/geometry/triangulation.h>

class BackendParams {

  public:

    BackendParams(const std::string& path);
    ~BackendParams();
    void loadYaml(const std::string& path);

    struct SegmentAccumulatorParams {
      uint32_t segment_size_;
    } seg_accumulator_;

    struct SegmentDatabaseParams {
      uint32_t max_segments_;
      size_t landmark_overlap_thresh_;
      uint8_t num_partitions_;
      std::string dir_;
      bool incremental_;
      double database_entropy_thresh_;
    } seg_database_;

    struct NoiseModelParams {
      double reproj_sigma_;

      bool stereo_use_epipolar_;
      bool epipolar_use_sampson_;
      double epipolar_sigma_;

      double odom_sigma_;
      double pose_prior_sigma_;
      double extrinsic_translation_sigma_;
    } noise_model_;

    struct MarginalCovarianceParams {
      double rotation_sigma_;
      double translation_sigma_;
    } marginal_covariance_;

    gtsam::FastMap<char, gtsam::Vector> relin_thresholds_;

    gtsam::TriangulationParameters triangulation_;

    // Mutators (some other params determined at runtime)
    void setNumPartitions(uint8_t num_partitions) {seg_database_.num_partitions_ = num_partitions;}
    void setDatabaseDirectory(const std::string& dir) {seg_database_.dir_ = dir;}

    // Accessors
    const SegmentAccumulatorParams& getSegmentAccumulatorParams() const {return seg_accumulator_;}
    const SegmentDatabaseParams& getSegmentDatabaseParams() const {return seg_database_;}
    const NoiseModelParams& getNoiseModelParams() const {return noise_model_;}
    const gtsam::FastMap<char, gtsam::Vector>& getRelinearizationParams() const {return relin_thresholds_;}
    const gtsam::TriangulationParameters& getTriangulationParams() const {return triangulation_;}
    const MarginalCovarianceParams& getMarginalCovarianceParams() const {return marginal_covariance_;}

};

#endif