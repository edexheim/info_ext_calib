#pragma once

#include "calibration/utils/basic_types.h"
#include "calibration/frontend/FrontendParams.h"
#include "calibration/frontend/FeatureTracker.h"
#include "calibration/frontend/FeatureDatabase.h"
#include "calibration/frontend/keyframe_selection/KeyframeSelecter.h"
#include "calibration/frontend/OutlierRejecter.h"
#include "calibration/stereo/stereo_params.h"

#include <thread>
#include <tbb/concurrent_queue.h>

#include <fstream>

class Frontend {

  public:

    typedef std::unique_ptr<Frontend> unique_ptr;

    Frontend(const FrontendParams& params);
    ~Frontend();
    void initialize();
    void join();

    // Input queues
    tbb::concurrent_bounded_queue<MultiFrameInput::shared_ptr> frame_queue_;
    // Output queue
    tbb::concurrent_bounded_queue<KeyframeInput::shared_ptr>* backend_queue_ = nullptr;
    tbb::concurrent_bounded_queue<MultiFrameInput::shared_ptr>* vis_queue_ = nullptr;

  private:

    void processingLoop();
    void handleFrame(const MultiFrameInput::shared_ptr& multi_frame);

    void kltDetectionTracking(const MultiFrameInput::shared_ptr& multi_frame);
    void orbDetectionTracking(const MultiFrameInput::shared_ptr& multi_frame);
    void kltOrbDetectionTracking(const MultiFrameInput::shared_ptr& multi_frame);

    std::unique_ptr<std::thread> processing_thread_;

    FrontendParams params_;

    FeatureTracker ft_tracker_;
    OutlierRejecter out_rej_;
    FeatureDatabase ft_database_;
    std::unique_ptr<KeyframeSelecter> kf_selecter_;

    size_t feature_id_;
    MultiFrameInput::shared_ptr prev_multi_frame_;
    gtsam::Pose3 T_wv_prev_;

    // Logging
    std::ofstream time_log_file;
};