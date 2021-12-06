#pragma once

#include "calibration/backend/SegmentDatabase.h"

class IncrementalSegmentDatabase : public SegmentDatabase {

  public:

    IncrementalSegmentDatabase(
        const BackendParams::SegmentDatabaseParams& database_params,
        const BackendParams::NoiseModelParams& noise_params,
        const gtsam::TriangulationParameters& triangulation_params,
        const gtsam::FastMap<char, gtsam::Vector> relin_params);

    ~IncrementalSegmentDatabase();

    std::vector<gtsam::Pose3> getExtrinsicsEstimates() const override;

    void retrieveEstimates(BackendOutput::shared_ptr& output_data) override;

    bool optimizeStep() override;

    // size_t constructGraphBatch( const std::vector<gtsam::Pose3>& extrinsics,
    //     gtsam::NonlinearFactorGraph& graph,
    //     gtsam::Values& values,
    //     std::vector<std::vector<gtsam::Key> >& pose_keys) const;

    // bool batchOptimize(
    //     const std::vector<gtsam::Pose3>& extrinsics,
    //     BackendOutput::shared_ptr& output_data) const;

  protected:

    void addCalibrationVariables(const std::vector<gtsam::Pose3>& extrinsics) override;

    void addSegment(const Segment& segment_to_add) override;

    void removeSegments(const std::set<SegmentId>& ids_to_remove) override;

    // Functions for incremental inference
    bool addToPartitions(const Segment& segment_to_add, bool& consecutive);

    void printPartitions() const;

    // Segment groupings (partitions)
    std::vector<std::set<SegmentId> > segment_partitions_;
    std::vector<gtsam::FactorIndex> partition_priors_;

    // Incremental inference
    gtsam::ISAM2Custom isam2_;
    gtsam::FactorIndex curr_factor_index_;
    std::map<SegmentId, gtsam::FactorIndices> segment_factor_indices_;
    std::map<gtsam::Key, std::map<SegmentId, gtsam::FactorIndices> > landmark_factor_indices_;
};
