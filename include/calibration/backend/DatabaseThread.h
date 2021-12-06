#pragma once

#include "calibration/utils/basic_types.h"
#include "calibration/backend/BackendParams.h"
#include "calibration/backend/SegmentAccumulator.h"
#include "calibration/backend/SegmentDatabase.h"
#include "calibration/stereo/stereo_calib.h"

#include <thread>
#include <tbb/concurrent_queue.h>

#include <fstream>

class DatabaseThread {

  public:

    typedef std::unique_ptr<DatabaseThread> unique_ptr;

    DatabaseThread(const BackendParams& params);
    ~DatabaseThread();
    bool loadDatabase(std::vector<gtsam::Pose3>& extrinsics);
    void initialize();
    void join();

    // Input queues
    tbb::concurrent_bounded_queue<Segment::shared_ptr> input_queue_;
    // Output queue
    tbb::concurrent_bounded_queue<BackendOutput::shared_ptr>* output_queue_ = nullptr;

  private:

    void processingLoop();
    void handleSegment(const Segment::shared_ptr& backend_input);

    std::unique_ptr<std::thread> processing_thread_;

    BackendParams params_;

    std::unique_ptr<SegmentDatabase> seg_database_; 

    bool converged_;

    // Logging
    std::ofstream time_log_file;
};