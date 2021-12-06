#pragma once

#include "calibration/utils/basic_types.h"
#include "calibration/backend/BackendParams.h"
#include "calibration/backend/SegmentAccumulator.h"
#include "calibration/backend/SegmentDatabase.h"
#include "calibration/stereo/stereo_calib.h"

#include <thread>
#include <tbb/concurrent_queue.h>

#include <fstream>

class SegmentConstruction {

  public:

    typedef std::unique_ptr<SegmentConstruction> unique_ptr;

    SegmentConstruction(const BackendParams& params, const stereo::Params& stereo_params);
    ~SegmentConstruction();
    void initialize();
    void join();

    // Input queue
    tbb::concurrent_bounded_queue<KeyframeInput::shared_ptr> input_queue_;
    // Output queue
    tbb::concurrent_bounded_queue<Segment::shared_ptr>* output_queue_ = nullptr;

  private:

    void processingLoop();
    void runStereo(KeyframeInput::shared_ptr keyframe_input);
    void handleKeyframe(KeyframeInput::shared_ptr keyframe_input);

    std::unique_ptr<std::thread> processing_thread_;

    std::unique_ptr<std::thread> stereo_thread_;

    BackendParams params_;
    stereo::Params stereo_params_;

    std::mutex seg_accumulator_mutex_;
    std::atomic<bool> stereo_running_;
    SegmentAccumulator seg_accumulator_;

    size_t keyframe_count_;

    // Logging
    std::ofstream keyframe_time_log_file;
    std::ofstream stereo_time_log_file;

};