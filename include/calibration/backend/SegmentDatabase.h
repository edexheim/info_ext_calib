#ifndef SEGMENTDATABASE_H
#define SEGMENTDATABASE_H

#include <vector>
#include <queue>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include "calibration/backend/Segment.h"
#include "calibration/backend/BackendParams.h"
// #include <gtsam/nonlinear/ISAM2.h>
#include "calibration/backend/ISAM2Custom.h"

class SegmentDatabase {

  public:

    SegmentDatabase(
        const BackendParams::SegmentDatabaseParams& database_params,
        const BackendParams::NoiseModelParams& noise_params,
        const gtsam::TriangulationParameters& triangulation_params);

    virtual ~SegmentDatabase();

    bool loadDatabase(std::vector<gtsam::Pose3>& extrinsics);

    bool saveDatabase() const;

    bool proposeSegment(Segment& new_segment);

    std::vector<gtsam::Pose3> getExtrinsicsEstimates(const gtsam::Values& values) const;

    virtual std::vector<gtsam::Pose3> getExtrinsicsEstimates() const = 0;

    void retrieveEstimates(const gtsam::Values& values, 
        BackendOutput::shared_ptr& output_data);

    virtual void retrieveEstimates(BackendOutput::shared_ptr& output_data) = 0;

    virtual bool optimizeStep() = 0;

    bool atCapacity() const {
      return segments_.size() >= database_params_.max_segments_;
    }

  protected:

    virtual void addCalibrationVariables(const std::vector<gtsam::Pose3>& extrinsics) = 0;

    virtual void addSegment(const Segment& segment_to_add) = 0;

    virtual void removeSegments(const std::set<SegmentId>& ids_to_remove) = 0;
    
    bool proposeSegmentPerPartition(
        const std::vector<double>& new_info_metrics,
        const SegmentId& new_segment_id,
        size_t& num_partitions_added,
        std::set<SegmentId>& segment_ids_to_remove);

    bool proposeSegmentMinMaxEntropy(
        const std::vector<double>& new_info_metrics,
        const std::vector<gtsam::Matrix3>& new_segment_marg_info,
        const SegmentId& new_segment_id,
        size_t& num_partitions_added,
        std::set<SegmentId>& segment_ids_to_remove);

    std::vector<double> getConservativeDatabaseLogDet();

    double getMaxDatabaseEntropy(int& q_max);

    bool isEnoughLandmarkOverlap(
        const Segment& segment1, 
        const Segment& segment2);

    gtsam::NonlinearFactor::shared_ptr createPriorFactor(
        size_t first_id, 
        unsigned char sess_char,
        const gtsam::Values& values);

    static unsigned char getSessionChar(uint8_t sess_id);
    
    // Calibration variables
    uint8_t num_cam_;
    gtsam::FastList<gtsam::Key> calib_keys_;
    gtsam::Values calib_values_init_;

    // Variables for assigning SegmentId
    size_t curr_sess_id_;
    size_t seq_id_;

    // Session variables
    std::map<gtsam::Key, gtsam::Key> rekey_map_;

    // Q max heaps of information content along with segment id
    std::vector<std::priority_queue<std::pair<double, SegmentId > > > info_heaps_;

    // Storage of all segments - avoids double counting
    std::map<SegmentId, size_t> segment_counts_;
    std::map<SegmentId, Segment> segments_;

    // Don't start optimizing until requirements met
    bool can_optimize_; 

    const BackendParams::SegmentDatabaseParams database_params_;
    const BackendParams::NoiseModelParams noise_params_; 
    const gtsam::TriangulationParameters triangulation_params_; 
    
};

#endif