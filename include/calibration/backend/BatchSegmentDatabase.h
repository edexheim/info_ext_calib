#pragma once

#include "calibration/backend/SegmentDatabase.h"
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

class BatchSegmentDatabase : public SegmentDatabase {

  public:

    BatchSegmentDatabase(
        const BackendParams::SegmentDatabaseParams& database_params,
        const BackendParams::NoiseModelParams& noise_params,
        const gtsam::TriangulationParameters& triangulation_params);

    ~BatchSegmentDatabase();

    std::vector<gtsam::Pose3> getExtrinsicsEstimates() const override;

    void retrieveEstimates(BackendOutput::shared_ptr& output_data) override;

    bool optimizeStep() override;

    size_t constructGraphBatch(
        gtsam::NonlinearFactorGraph& graph,
        gtsam::Values& values) const;

    // bool batchOptimize(
    //     const std::vector<gtsam::Pose3>& extrinsics,
    //     BackendOutput::shared_ptr& output_data) const;

  protected:

    void addCalibrationVariables(const std::vector<gtsam::Pose3>& extrinsics) override;

    void addSegment(const Segment& segment_to_add) override;

    void removeSegments(const std::set<SegmentId>& ids_to_remove) override;

    std::unique_ptr<gtsam::LevenbergMarquardtOptimizer> optimizer_;
    gtsam::LevenbergMarquardtParams optimizer_params_; 
    
};
