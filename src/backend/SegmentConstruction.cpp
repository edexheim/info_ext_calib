#include "calibration/backend/SegmentConstruction.h"
#include "calibration/backend/BatchSegmentDatabase.h"
#include "calibration/backend/IncrementalSegmentDatabase.h"

SegmentConstruction::SegmentConstruction(const BackendParams& params,
    const stereo::Params& stereo_params) 
    : params_(params), stereo_params_(stereo_params),
      seg_accumulator_(params.getSegmentAccumulatorParams(),
                      params.getNoiseModelParams()),
      stereo_running_(false),
      keyframe_count_(0)
{

}

SegmentConstruction::~SegmentConstruction() {
  this->join();
}

void SegmentConstruction::initialize() {
  keyframe_time_log_file.open("../logs/timing/keyframe.txt", std::ofstream::out | std::ofstream::trunc);
  stereo_time_log_file.open("../logs/timing/stereo.txt", std::ofstream::out | std::ofstream::trunc);

  processing_thread_ = std::make_unique<std::thread>(&SegmentConstruction::processingLoop, this);
}

void SegmentConstruction::processingLoop() {

  KeyframeInput::shared_ptr keyframe_input;
  while (true) {
    if (!input_queue_.empty()) {
      input_queue_.pop(keyframe_input);
      if (keyframe_input.get()) {
        this->handleKeyframe(keyframe_input);
      }
      else {
        break;
      }
    }
  }

  if (output_queue_) output_queue_->push(nullptr);
}

void SegmentConstruction::join() {
  if (stereo_thread_.get() && (stereo_running_ || stereo_thread_->joinable()) ) {
    stereo_thread_->join();
  }

  if (processing_thread_->joinable()) {
    processing_thread_->join();
  }

  keyframe_time_log_file.close();
  stereo_time_log_file.close();
}

void SegmentConstruction::runStereo(KeyframeInput::shared_ptr keyframe_input) {
  auto t1 = std::chrono::high_resolution_clock::now();

  size_t frame_id = keyframe_input->frame_id_;

  std::vector<std::pair<size_t, size_t> > landmark_landmark_matches;
  std::vector<std::pair<size_t, StereoMatch> > landmark_point_matches;
  std::vector<std::pair<StereoMatch, StereoMatch> > point_point_matches;
  stereo::setupCalib(
      keyframe_input->intrinsics_, keyframe_input->extrinsics_, 
      keyframe_input->frames_,
      landmark_landmark_matches, landmark_point_matches, point_point_matches,
      stereo_params_);

  // Add stereo id matches
  seg_accumulator_mutex_.lock();
  seg_accumulator_.addLandmarkLandmarkMatches(landmark_landmark_matches);
  seg_accumulator_.addLandmarkPointMatches(
      frame_id, keyframe_input->intrinsics_, landmark_point_matches);
  seg_accumulator_.addPointPointMatches(
      frame_id, keyframe_input->intrinsics_, point_point_matches);
  seg_accumulator_mutex_.unlock();

  stereo_running_ = false;

  auto t2 = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> stereo_time = t2 - t1;

  stereo_time_log_file << stereo_time.count() << std::endl;

  // std::cout << "Stereo duration: " << stereo_time.count() << " s" << std::endl;
}

void SegmentConstruction::handleKeyframe(KeyframeInput::shared_ptr keyframe_input) {

  // gttic_(Backend_handleKeyframe);

  auto t1 = std::chrono::high_resolution_clock::now();

  // Stereo matching on keyframe
  if (stereo_params_.use_stereo_ &&
      keyframe_count_ % stereo_params_.keyframe_skip_ == 0) {
    // gttic_(stereo);
    if (stereo_thread_.get() && (stereo_running_ || stereo_thread_->joinable()) ) {
      stereo_thread_->join();
    }

    stereo_running_ = true;
    stereo_thread_ = std::make_unique<std::thread>(&SegmentConstruction::runStereo, this, keyframe_input);
    // gttoc_(stereo);
  }

  auto t2 = std::chrono::high_resolution_clock::now();
  // gttic_(add_to_segment);
  // Handle monocular features
  seg_accumulator_mutex_.lock();
  bool segment_full = seg_accumulator_.handleKeyframe(keyframe_input);
  seg_accumulator_mutex_.unlock();
  // gttoc_(add_to_segment);
  auto t3 = std::chrono::high_resolution_clock::now();

  // Check if new segment
  if (segment_full) {
    // gttic_(snew_segment);
    if (stereo_thread_.get() && (stereo_running_ || stereo_thread_->joinable()) ) {
      stereo_thread_->join();
    }

    Segment::shared_ptr new_segment = std::make_shared<Segment>(seg_accumulator_.generateSegment());
    if (output_queue_) output_queue_->push(new_segment);
    seg_accumulator_.reset();
    // gttoc_(new_segment);
  }

  keyframe_count_++;

  auto t4 = std::chrono::high_resolution_clock::now();
  // gttoc_(Backend_handleKeyframe);
  // gtsam::tictoc_print_();

  std::chrono::duration<double> total_time = t4 - t1;
  std::chrono::duration<double> seg_acc_time = t3 - t2;
  std::chrono::duration<double> push_data_time = t4 - t3;

  keyframe_time_log_file << total_time.count() << " " << seg_acc_time.count() << " " << push_data_time.count() << std::endl;

  // std::cout << "SegmentConstruction::handleKeyframe duration: " << total_time.count() << " s" << std::endl;
  // std::cout << "SegmentConstruction::addKeyframeToSegment duration: " << seg_acc_time.count() << " s" << std::endl;
  // std::cout << "SegmentConstruction::pushSegment duration: " << push_data_time.count() << " s" << std::endl;
}