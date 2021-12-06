#include "calibration/backend/DatabaseThread.h"
#include "calibration/backend/BatchSegmentDatabase.h"
#include "calibration/backend/IncrementalSegmentDatabase.h"

DatabaseThread::DatabaseThread(const BackendParams& params) 
    : params_(params),
      converged_(true)
{
  if (params.seg_database_.incremental_) {
    seg_database_ = std::make_unique<IncrementalSegmentDatabase>(
        params.getSegmentDatabaseParams(),
        params.getNoiseModelParams(),
        params.getTriangulationParams(),
        params.getRelinearizationParams() );
  }
  else {
    seg_database_ = std::make_unique<BatchSegmentDatabase>(
        params.getSegmentDatabaseParams(),
        params.getNoiseModelParams(),
        params.getTriangulationParams() );
  }
}

DatabaseThread::~DatabaseThread() {
  this->join();
}

bool DatabaseThread::loadDatabase(std::vector<gtsam::Pose3>& extrinsics) {
  return seg_database_->loadDatabase(extrinsics);
}

void DatabaseThread::initialize() {
  time_log_file.open("../logs/timing/database.txt", std::ofstream::out | std::ofstream::trunc);

  processing_thread_ = std::make_unique<std::thread>(&DatabaseThread::processingLoop, this);
}

void DatabaseThread::processingLoop() {

  Segment::shared_ptr segment_input;
  while (true) {
    if (!input_queue_.empty()) {
      input_queue_.pop(segment_input);
      if (segment_input.get()) {
        this->handleSegment(segment_input);
      }
      else {
        break;
      }
    }
    else {
      if (!converged_) {
        // std::cout << "Optimizing" << std::endl;
        converged_ = seg_database_->optimizeStep();

        if (converged_) {
          std::cout << "CONVERGED" << std::endl;

          // if (output_queue_) {
          //   BackendOutput::shared_ptr output_data(new BackendOutput);
          //   seg_database_->retrieveEstimates(output_data);
          //   output_queue_->push(output_data);
          // }
        }
      }
    }
  }

  while (!converged_) {
    converged_ = seg_database_->optimizeStep();
  }
  if (output_queue_) {
    BackendOutput::shared_ptr output_data(new BackendOutput);
    seg_database_->retrieveEstimates(output_data);
    output_queue_->push(output_data);
  }

  seg_database_->saveDatabase();
  if (output_queue_) output_queue_->push(nullptr);
}

void DatabaseThread::join() {
  if (processing_thread_->joinable()) {
    processing_thread_->join();
  }

  time_log_file.close();
}

void DatabaseThread::handleSegment(const Segment::shared_ptr& segment_input) {

  auto t1 = std::chrono::high_resolution_clock::now();
  segment_input->calcInfoContent(
        seg_database_->getExtrinsicsEstimates(), // use current estimate for linearization
        params_.getNoiseModelParams(),
        params_.getTriangulationParams(),
        params_.getMarginalCovarianceParams() );

  auto t2 = std::chrono::high_resolution_clock::now();

  bool is_new_seg = seg_database_->proposeSegment(*segment_input);

  auto t3 = std::chrono::high_resolution_clock::now();

  std::cout << "Segment " << segment_input->getId() << " new segment? " << is_new_seg << std::endl;
  if (is_new_seg) {
    converged_ = false;
  //   while (!converged_) {
      // converged_ = seg_database_->optimizeStep();
  //   }
  }

  auto t4 = std::chrono::high_resolution_clock::now();

  if (output_queue_) {
    BackendOutput::shared_ptr output_data(new BackendOutput);
    seg_database_->retrieveEstimates(output_data);
    // output_queue_->push(output_data);
    output_queue_->try_push(output_data);
  }

  auto t5 = std::chrono::high_resolution_clock::now();

  std::chrono::duration<double> total_time = t5 - t1;
  std::chrono::duration<double> info_time = t2 - t1;
  std::chrono::duration<double> propose_segment_time = t3 - t2;
  std::chrono::duration<double> optimize_step_time = t4 - t3;

  time_log_file << total_time.count() << " " << info_time.count() << " " << propose_segment_time.count() << " " << optimize_step_time.count() << std::endl;
  // std::cout << "DatabaseThread::handleSegment duration: " << total_time.count() << " s" << std::endl;
  // std::cout << "DatabaseThread::handleSegment calcInfo duration: " << info_time.count() << " s" << std::endl;
  // if  (is_new_seg) {
  //   std::cout << "DatabaseThread::handleSegment proposeSegment duration: " << propose_segment_time.count() << " s" << std::endl;
  // }
  // std::cout << "DatabaseThread::handleSegment optimizeStep duration: " << optimize_step_time.count() << " s" << std::endl;
}