#include "calibration/frontend/FeatureTracker.h"
#include "calibration/frontend/RotationPredictor.h"
#include <opencv2/video/tracking.hpp>

// only for debugging
#include <iostream>

FeatureTracker::FeatureTracker(const FrontendParams::TrackerParams& params)
    : params_(params) 
  {}

FeatureTracker::~FeatureTracker() {

}

// TODO: Many lists and vectors here, check if performance is an issue
std::list<size_t> FeatureTracker::track(const Frame* frame_prev, 
                                        Frame* frame_next, 
                                        const gtsam::PinholeModel* calib,
                                        const OutlierRejecter& out_rej,
                                        const gtsam::Pose3& T_c2_c1) const {

  std::list<size_t> invalid_ids;

  if (frame_prev && frame_prev->features_.size() > 0) {

    // Feature prediction
    RotationPredictor feature_pred(calib, T_c2_c1.rotation(), 
        frame_next->img_.size());
    std::vector<cv::Point2f> points_next = feature_pred.predict(frame_prev->features_);
    // std::vector<cv::Point2f> points_next = frame_prev->features_; 

    // std::cout << "Features before tracking: " << frame_prev->features_.size() << std::endl;

    // KLT Tracking
    std::vector<uchar> tracked_status;
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(frame_prev->img_, frame_next->img_, 
        frame_prev->features_, points_next,
        tracked_status, err, 
        cv::Size(params_.window_size_, params_.window_size_), params_.max_level_, 
        cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 
            params_.max_iter_, params_.convergence_eps_),
        cv::OPTFLOW_USE_INITIAL_FLOW, params_.min_eig_thresh_ );

    // Store successfully tracked keypoints
    std::list<size_t> valid_ids;
    std::vector<cv::Point2f> tracked_prev;
    std::vector<cv::Point2f> tracked_next;
    for (int i=0; i<tracked_status.size(); i++) {
      if (tracked_status[i]) {
        tracked_prev.push_back(frame_prev->features_[i]);
        tracked_next.push_back(points_next[i]);
        valid_ids.push_back(frame_prev->feature_ids_[i]);
      }
      else {
        invalid_ids.push_back(frame_prev->feature_ids_[i]);
      }
    }

    if (params_.two_way_tracking_ && tracked_prev.size() > 0) {
      // std::cout << "Num tracked features before two way tracking: " 
      //     << tracked_prev.size() << std::endl;
      double thresh_sq = params_.two_way_thresh_ * params_.two_way_thresh_;
      std::vector<uchar> tracked_status2;
      std::vector<cv::Point2f> points_prev; // = tracked_prev;
      std::vector<float> err2;
      cv::calcOpticalFlowPyrLK(frame_next->img_, frame_prev->img_, 
          tracked_next, points_prev, 
          tracked_status2, err2, 
          cv::Size(params_.window_size_, params_.window_size_), params_.max_level_, 
          cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 
              params_.max_iter_, params_.convergence_eps_),
          0 /*no prediction*/, params_.min_eig_thresh_ );
      auto id_it = valid_ids.begin();
      auto tracked_prev_it = tracked_prev.begin();
      auto tracked_next_it = tracked_next.begin();
      for (int i=0; i<tracked_status2.size(); i++) {
        double x_diff = tracked_prev_it->x - points_prev[i].x;
        double y_diff = tracked_prev_it->y - points_prev[i].y;
        double dist_sq = x_diff*x_diff + y_diff*y_diff;

        if ( tracked_status2[i] && (dist_sq < thresh_sq) ) {
          id_it++;
          tracked_prev_it++;
          tracked_next_it++;
        }
        else {
          invalid_ids.push_back(*id_it);
          id_it = valid_ids.erase(id_it);
          tracked_prev_it = tracked_prev.erase(tracked_prev_it);
          tracked_next_it = tracked_next.erase(tracked_next_it);
        }
      }
    }

    // std::cout << "Num tracked features: " << tracked_prev.size() << std::endl;

    // TODO: How to handle more generally with 2D-2D, 3D-2D, 3D-3D?
    std::vector<uchar> inlier_status 
      = out_rej.getInliers(tracked_prev, tracked_next, calib);

    if (inlier_status.size() > 0) { // size is 0 if not enough tracked points
      int i = 0;
      auto it = valid_ids.begin();
      while (it != valid_ids.end()) {
        if (inlier_status[i]) {
          frame_next->features_.push_back(tracked_next[i]);
          frame_next->feature_ids_.push_back(*it);
          it++;
        }
        else {
          invalid_ids.push_back(*it);
          it = valid_ids.erase(it);
        }
        i++;
      }
    }
    // std::cout << "inlier features: " 
      // << frame_next->features_.size() << std::endl;
  }
  return invalid_ids;
}