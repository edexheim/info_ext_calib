#include "calibration/frontend/Frontend.h"
#include "calibration/frontend/keypoints.h"

#include "calibration/frontend/keyframe_selection/EntropyKeyframeSelecter.h"
#include "calibration/frontend/keyframe_selection/FeatureKeyframeSelecter.h"
#include "calibration/frontend/keyframe_selection/MotionKeyframeSelecter.h"

// For ORB matching
#include "calibration/stereo/stereo_matching.h"
#include "calibration/frontend/RotationPredictor.h"
#include <opencv2/video/tracking.hpp>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include <chrono>
#include <fstream>

Frontend::Frontend(
    const FrontendParams& params)
    : params_(params),
      ft_tracker_(params.getTrackerParams()),
      out_rej_(params.getOutRejParams()),
      ft_database_(),
      feature_id_(1) 
{
  const auto& kf_params = params.getKeyframeSelecterParams();
  switch (kf_params.keyframe_type_) {
    case FrontendParams::KeyframeSelecterParams::Entropy:
      kf_selecter_ = std::make_unique<EntropyKeyframeSelecter>(kf_params);
      break;
    case FrontendParams::KeyframeSelecterParams::Feature:
      kf_selecter_ = std::make_unique<FeatureKeyframeSelecter>(kf_params);
      break;  
    case FrontendParams::KeyframeSelecterParams::Motion:
      kf_selecter_ = std::make_unique<MotionKeyframeSelecter>(kf_params);
      break;
    default:
      throw std::runtime_error("Invalid keyframe selecter type");
  }
}

Frontend::~Frontend() {
  this->join();
}

void Frontend::initialize() {
  time_log_file.open("../logs/timing/frontend.txt", std::ofstream::out | std::ofstream::trunc);

  processing_thread_ = std::make_unique<std::thread>(&Frontend::processingLoop, this);
}

void Frontend::processingLoop() {

  MultiFrameInput::shared_ptr multi_frame;
  while (true) {
    frame_queue_.pop(multi_frame);
    if (multi_frame.get()) {
      this->handleFrame(multi_frame);
    }
    else {
      break;
    }
  }
  if (backend_queue_) backend_queue_->push(nullptr);
  if (vis_queue_) vis_queue_->push(nullptr);
}

void Frontend::join() {
  if (processing_thread_->joinable()) {
    processing_thread_->join();
  }

  time_log_file.close();
}

void Frontend::handleFrame(const MultiFrameInput::shared_ptr& multi_frame) {

  // gttic_(Frontend_handleKeyframe);

  auto t1 = std::chrono::high_resolution_clock::now();

  size_t NUM_CAM = multi_frame->frames_.size();
  size_t frame_id = multi_frame->frame_id_;
  const gtsam::Pose3& T_wv = multi_frame->T_wv_est_;
  gtsam::Pose3 T_v2_v1 = T_wv.inverse() * T_wv_prev_;


  // gttic_(tracking);
  // Detection/tracking/matching (order dependent on type)
  if (params_.tracker_type_ == FrontendParams::TrackerType::KLT) {
    this->kltDetectionTracking(multi_frame);
  }
  else if (params_.tracker_type_ == FrontendParams::TrackerType::ORB) {
    this->orbDetectionTracking(multi_frame);
  }
  else if (params_.tracker_type_ == FrontendParams::TrackerType::KLT_ORB) {
    this->kltOrbDetectionTracking(multi_frame);
  }
  else {
    throw std::runtime_error("Frontend: specified tracker_type not implemented");
  }
  // gttoc_(tracking);
  auto t2 = std::chrono::high_resolution_clock::now();

  // gttic_(get_database_matches);
  // Keyframe selection
  std::vector<std::vector<FeatureReference> > active_refs(NUM_CAM);
  std::vector<std::vector<cv::Point2f> > active_features(NUM_CAM);
  std::vector<std::vector<FeatureReference> > inactive_refs(NUM_CAM);
  std::vector<std::vector<cv::Point2f> > inactive_features(NUM_CAM);
  for (int l=0; l<NUM_CAM; l++) {
    // Active features have valid keyframe match
    // Inactive features have matched since last keyframe
    const auto& frame = multi_frame->frames_[l];
    ft_database_.getMatches(frame.get(),
                            active_refs[l], active_features[l],
                            inactive_refs[l], inactive_features[l]);
  }
  // gttoc_(get_database_matches);

  // gttic_(keyframe_check);
  bool new_kf = kf_selecter_->checkNewKeyframe(
      frame_id, T_wv,
      multi_frame->intrinsics_, multi_frame->extrinsics_,
      active_refs, active_features,
      inactive_refs, inactive_features);
  // gttoc_(keyframe_check);

  auto t3 = std::chrono::high_resolution_clock::now();

  if (new_kf) {
    ft_database_.addKeyframeId(frame_id);
  }
  ft_database_.addFeatures(multi_frame);

  // Push new features and pose estimate to backend
  if (new_kf && backend_queue_) {
    // gttic_(creating_keyframe_output);
    KeyframeInput::shared_ptr output_data(new KeyframeInput);
    output_data->frame_id_ = multi_frame->frame_id_;
    // TODO: Do we need to copy frames?
    output_data->frames_ = multi_frame->frames_;
    output_data->T_wv_est_ = multi_frame->T_wv_est_;
    output_data->intrinsics_ = multi_frame->intrinsics_;
    output_data->extrinsics_ = multi_frame->extrinsics_;

    backend_queue_->push(output_data);
    // gttoc_(creating_keyframe_output);
  }

  // gttic_(creating_viz_output);
  // TODO: Do we need to copy instead of passing pointer?
  T_wv_prev_ = T_wv;
  // prev_multi_frame_ = std::make_shared<MultiFrameInput>(*multi_frame);
  prev_multi_frame_ = multi_frame;
  if (vis_queue_)
    vis_queue_->try_push(multi_frame);
    // vis_queue_->push(std::make_shared<MultiFrameInput>(*multi_frame));
  // gttoc_(creating_viz_output);

  // gttoc_(Frontend_handleKeyframe);
  // gtsam::tictoc_print_();
  auto t4 = std::chrono::high_resolution_clock::now();

  std::chrono::duration<double> total_time = t4 - t1;
  std::chrono::duration<double> feature_tracking_time = t2 - t1;
  std::chrono::duration<double> keyframe_selection_time = t3 - t2;
  std::chrono::duration<double> push_data_time = t4 - t3;

  time_log_file << total_time.count() << " " << feature_tracking_time.count() << " " << keyframe_selection_time.count() << " " << push_data_time.count() << std::endl;
  // std::cout << "Frontend::handleFrame duration: " << total_time.count() << " s" << std::endl;
  // std::cout << "Frontend::featureTracking duration: " << feature_tracking_time.count() << " s" << std::endl;
  // std::cout << "Frontend::selectKeyframe duration: " << keyframe_selection_time.count() << " s" << std::endl;
  // std::cout << "Frontend::pushData duration: " << push_data_time.count() << " s" << std::endl;me << " " << keyframe_selection_time << " " << push_data_time << std::endl;
}


void Frontend::kltDetectionTracking(
      const MultiFrameInput::shared_ptr& multi_frame) {

  size_t NUM_CAM = multi_frame->frames_.size();
  const gtsam::Pose3& T_wv = multi_frame->T_wv_est_;
  gtsam::Pose3 T_v2_v1 = T_wv.inverse() * T_wv_prev_;

  // gttic_(tracking);
  // Feature tracking
  if (prev_multi_frame_.get()) {
    for (int l=0; l<NUM_CAM; l++) {
      const gtsam::Pose3& T_vc_est = multi_frame->extrinsics_[l];
      gtsam::Pose3 T_c2_c1 = T_vc_est.inverse() * T_v2_v1 * T_vc_est;
      const gtsam::PinholeModel::shared_ptr& intrinsics = multi_frame->intrinsics_[l];
      const auto& prev_frame = prev_multi_frame_->frames_[l];
      const auto& frame = multi_frame->frames_[l];
      std::list<size_t> invalid_ids = ft_tracker_.track(
          prev_frame.get(), frame.get(), 
          intrinsics.get(), out_rej_, T_c2_c1);

      ft_database_.removeFeatures(invalid_ids);
    }
  }
  // gttoc_(tracking);

  // gttic_(detection);
  // Feature detection 
  const FrontendParams::DetectorParams& detector_params 
      = params_.getDetectorParams();
  tbb::parallel_for(tbb::blocked_range<size_t>(0, NUM_CAM),
      [&](const tbb::blocked_range<size_t>& range_l) {
  for (size_t l = range_l.begin(); l != range_l.end(); l++) {
  // for (int l=0; l<NUM_CAM; l++) {
    auto& frame = multi_frame->frames_[l];
    const auto& mask = multi_frame->masks_[l];
    std::vector<cv::Point2f> new_features = 
        keypoints::detectFeatures(frame->img_, frame->features_,
            mask,
            detector_params.grid_size_, 
            detector_params.num_features_per_cell_,
            detector_params.min_dist_);
    frame->addNewFeatures(new_features, feature_id_);
  // }
  } });
  // gttoc_(detection);
}


void Frontend::orbDetectionTracking(
    const MultiFrameInput::shared_ptr& multi_frame) {

  size_t NUM_CAM = multi_frame->frames_.size();
  const gtsam::Pose3& T_wv = multi_frame->T_wv_est_;
  gtsam::Pose3 T_v2_v1 = T_wv.inverse() * T_wv_prev_;

  const FrontendParams::DetectorParams& detector_params 
      = params_.getDetectorParams();

  for (int l=0; l<NUM_CAM; l++) {
    auto& frame = multi_frame->frames_[l];
    // Detection
    const auto& mask = multi_frame->masks_[l];
    std::vector<cv::Point2f> new_features = 
        keypoints::detectFeatures(frame->img_, frame->features_,
            mask,
            detector_params.grid_size_, 
            detector_params.num_features_per_cell_,
            detector_params.min_dist_);

    frame->features_ = new_features;
    frame->feature_ids_.resize(new_features.size(), 0);

    // Description
    std::vector<cv::KeyPoint> keypoints;
    cv::KeyPoint::convert(new_features, keypoints);
    // std::cout << "Before: " << keypoints.size() << std::endl;
    keypoints::describeKeypointsORB(frame->img_, keypoints, frame->descriptors_);
    // std::cout << "After: " << keypoints.size() << std::endl;

    // Matching
    if (prev_multi_frame_.get()) {
      const auto& prev_frame = prev_multi_frame_->frames_[l];
      // Convert points to rays
      const gtsam::PinholeModel::shared_ptr& intrinsics = multi_frame->intrinsics_[l];
      std::vector<gtsam::Point3> rays_prev, rays_next;
      rays_prev.reserve(prev_frame->features_.size());
      for (int i = 0; i<prev_frame->features_.size(); i++) {
        gtsam::Point2 p_u = intrinsics->calibrate(gtsam::Point2(prev_frame->features_[i].x, prev_frame->features_[i].y));
        gtsam::Point3 r(p_u.x(), p_u.y(), 1.0);  
        rays_prev.push_back(r);
      }

      rays_next.reserve(frame->features_.size());
      for (int i = 0; i<frame->features_.size(); i++) {
        gtsam::Point2 p_u = intrinsics->calibrate(gtsam::Point2(frame->features_[i].x, frame->features_[i].y));
        gtsam::Point3 r(p_u.x(), p_u.y(), 1.0);  
        rays_next.push_back(r);
      }

      // TODO: Hardcoded parameters 

      // Create match masks based on sampson distance
      const gtsam::Pose3& T_vc_est = multi_frame->extrinsics_[l];
      gtsam::Pose3 T_c1_c2 = T_vc_est.inverse() * T_v2_v1.inverse() * T_vc_est;
      cv::Mat epipolar_mask;
      stereo::getSampsonMask(rays_prev, rays_next, T_c1_c2, epipolar_mask, 5e-1);
      // cv::Mat epipolar_mask(rays_prev.size(), rays_next.size(), CV_8UC1, 255);

      // Match features with cross-check and distance ratio
      std::vector<cv::DMatch> matches;
      stereo::matchFeatures(prev_frame->descriptors_, frame->descriptors_,
          epipolar_mask, matches, 0.8);

      // std::cout << "Matches: " << matches.size() << std::endl;
      // Get correspondences
      std::vector<cv::Point2f> tracked_prev;
      std::vector<cv::Point2f> tracked_next;
      for (const auto& match : matches) {
        tracked_prev.push_back(prev_frame->features_[match.queryIdx]);
        tracked_next.push_back(frame->features_[match.trainIdx]);
      }
      // std::cout << "Inliers: " << tracked_prev.size() << std::endl;

      // 5-point RANSAC
      std::vector<uchar> inlier_status 
        = out_rej_.getInliers(tracked_prev, tracked_next, intrinsics.get());
      std::set<size_t> inlier_ids;
      for (int i = 0; i < matches.size(); i++) {
        const auto& match = matches[i];
        size_t prev_id = prev_frame->feature_ids_[match.queryIdx];
        if (inlier_status[i]) {
          frame->feature_ids_[match.trainIdx] =  prev_id;
          inlier_ids.insert(prev_id);
        }
      }
      // Remove invalid ids from database
      std::list<size_t> invalid_ids;
      for (const auto& id : prev_frame->feature_ids_) {
        if (inlier_ids.find(id) == inlier_ids.end()) {
          invalid_ids.push_back(id);
        }
      }
      ft_database_.removeFeatures(invalid_ids);
    } 

    // Assign new feature ids
    for (auto& id : frame->feature_ids_) {
      if (id == 0) {
        id = feature_id_;
        feature_id_++;
      }
    }
    // End matchcing for camera
  }
}

void Frontend::kltOrbDetectionTracking(
    const MultiFrameInput::shared_ptr& multi_frame) {

  size_t NUM_CAM = multi_frame->frames_.size();
  const gtsam::Pose3& T_wv = multi_frame->T_wv_est_;
  gtsam::Pose3 T_v2_v1 = T_wv.inverse() * T_wv_prev_;

  for (int l=0; l<NUM_CAM; l++) {
    const auto& frame = multi_frame->frames_[l];
    
    const gtsam::PinholeModel::shared_ptr& intrinsics = multi_frame->intrinsics_[l];

    std::vector<size_t> klt_valid_ids;
    std::vector<size_t> klt_invalid_ids;

    std::vector<cv::Point2f> tracked_prev;
    std::vector<cv::Point2f> tracked_next;
    std::vector<size_t> tracked_ids;
    // 1. KLT
    if (prev_multi_frame_.get()) {
      const FrontendParams::TrackerParams& tracker_params 
          = params_.getTrackerParams();


      const gtsam::Pose3& T_vc_est = multi_frame->extrinsics_[l];
      gtsam::Pose3 T_c2_c1 = T_vc_est.inverse() * T_v2_v1 * T_vc_est;
      const auto& frame_prev = prev_multi_frame_->frames_[l];
      
      // Feature prediction
      RotationPredictor feature_pred(intrinsics.get(), T_c2_c1.rotation(), 
          frame->img_.size());
      std::vector<cv::Point2f> points_next = feature_pred.predict(frame_prev->features_);
      // std::vector<cv::Point2f> points_next = frame_prev->features_; 
      // std::cout << "Features before tracking: " << frame_prev->features_.size() << std::endl;

      // KLT Tracking
      std::vector<uchar> tracked_status;
      std::vector<float> err;
      cv::calcOpticalFlowPyrLK(frame_prev->img_, frame->img_, 
          frame_prev->features_, points_next,
          tracked_status, err, 
          cv::Size(tracker_params.window_size_, tracker_params.window_size_), tracker_params.max_level_, 
          cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 
              tracker_params.max_iter_, tracker_params.convergence_eps_),
          cv::OPTFLOW_USE_INITIAL_FLOW, tracker_params.min_eig_thresh_ );

      // Store successfully tracked keypoints
      for (int i=0; i<tracked_status.size(); i++) {
        if (tracked_status[i]) {
          tracked_prev.push_back(frame_prev->features_[i]);
          tracked_next.push_back(points_next[i]);
          tracked_ids.push_back(frame_prev->feature_ids_[i]);
          klt_valid_ids.push_back(i);
        }
        else {
          klt_invalid_ids.push_back(i);
        }
      }

      if (tracker_params.two_way_tracking_ && tracked_prev.size() > 0) {
        double thresh_sq = tracker_params.two_way_thresh_ * tracker_params.two_way_thresh_;
        std::vector<uchar> tracked_status2;
        std::vector<cv::Point2f> points_prev; // = tracked_prev;
        std::vector<float> err2;
        cv::calcOpticalFlowPyrLK(frame->img_, frame_prev->img_, 
            tracked_next, points_prev, 
            tracked_status2, err2, 
            cv::Size(tracker_params.window_size_, tracker_params.window_size_), tracker_params.max_level_, 
            cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 
                tracker_params.max_iter_, tracker_params.convergence_eps_),
            0 /*no prediction*/, tracker_params.min_eig_thresh_ );
        auto id_it = klt_valid_ids.begin();
        auto tracked_prev_it = tracked_prev.begin();
        auto tracked_next_it = tracked_next.begin();
        auto tracked_id_it = tracked_ids.begin();
        for (int i=0; i<tracked_status2.size(); i++) {
          double x_diff = tracked_prev_it->x - points_prev[i].x;
          double y_diff = tracked_prev_it->y - points_prev[i].y;
          double dist_sq = x_diff*x_diff + y_diff*y_diff;

          if ( tracked_status2[i] && (dist_sq < thresh_sq) ) {
            id_it++;
            tracked_prev_it++;
            tracked_next_it++;
            tracked_id_it++;
          }
          else {
            klt_invalid_ids.push_back(*id_it);
            id_it = klt_valid_ids.erase(id_it);
            tracked_prev_it = tracked_prev.erase(tracked_prev_it);
            tracked_next_it = tracked_next.erase(tracked_next_it);
            tracked_id_it = tracked_ids.erase(tracked_id_it);
          }
        }
      }

    }


    // 2. Detect new features on current frame (some exisitng features due to klt)
    const FrontendParams::DetectorParams& detector_params 
      = params_.getDetectorParams();
    // Detection
    const auto& mask = multi_frame->masks_[l];
    std::vector<cv::Point2f> new_features = 
        keypoints::detectFeatures(frame->img_, frame->features_,
            mask,
            detector_params.grid_size_, 
            detector_params.num_features_per_cell_,
            detector_params.min_dist_);



    // 4. ORB matching with previous frame (only on points not tracked!)
    if (prev_multi_frame_.get()) {

      const auto& frame_prev = prev_multi_frame_->frames_[l];

      // Get previous frame features that failed to track
      std::vector<cv::Point2f> orb_features_prev;
      cv::Mat orb_descs_prev(klt_invalid_ids.size(), frame_prev->descriptors_.cols, frame_prev->descriptors_.type());
      // for (const auto& id : klt_invalid_ids) {
      for (int i = 0; i < klt_invalid_ids.size(); i++) {
        const auto& id = klt_invalid_ids[i];
        orb_features_prev.push_back(frame_prev->features_[id]);
        orb_descs_prev.row(i) = frame_prev->descriptors_.row(i).clone();
      }

      // Get current frame features that are new
      std::vector<cv::KeyPoint> keypoints_curr;
      cv::KeyPoint::convert(new_features, keypoints_curr);
      cv::Mat orb_descs_curr;
      keypoints::describeKeypointsORB(frame->img_, keypoints_curr, orb_descs_curr);

      // Convert points to rays
      std::vector<gtsam::Point3> rays_prev, rays_next;
      rays_prev.reserve(orb_features_prev.size());
      for (int i = 0; i<orb_features_prev.size(); i++) {
        gtsam::Point2 p_u = intrinsics->calibrate(gtsam::Point2(orb_features_prev[i].x, orb_features_prev[i].y));
        gtsam::Point3 r(p_u.x(), p_u.y(), 1.0);  
        rays_prev.push_back(r);
      }

      rays_next.reserve(new_features.size());
      for (int i = 0; i<new_features.size(); i++) {
        gtsam::Point2 p_u = intrinsics->calibrate(gtsam::Point2(new_features[i].x, new_features[i].y));
        gtsam::Point3 r(p_u.x(), p_u.y(), 1.0);  
        rays_next.push_back(r);
      }

      // TODO: Hardcoded parameters 

      // Create match masks based on sampson distance
      const gtsam::Pose3& T_vc_est = multi_frame->extrinsics_[l];
      gtsam::Pose3 T_c1_c2 = T_vc_est.inverse() * T_v2_v1.inverse() * T_vc_est;
      cv::Mat epipolar_mask;
      stereo::getSampsonMask(rays_prev, rays_next, T_c1_c2, epipolar_mask, 5e-1);
      // cv::Mat epipolar_mask(rays_prev.size(), rays_next.size(), CV_8UC1, 255);

      // Match features with cross-check and distance ratio
      std::vector<cv::DMatch> matches;
      stereo::matchFeatures(orb_descs_prev, orb_descs_curr,
          epipolar_mask, matches, 0.8);

      // std::cout << "Matches: " << matches.size() << std::endl;
      // Get correspondences
      for (int i = 0; i < matches.size(); i++) {
        const auto& match = matches[i];
        tracked_prev.push_back(orb_features_prev[match.queryIdx]);
        tracked_next.push_back(new_features[match.trainIdx]);
        tracked_ids.push_back(frame_prev->feature_ids_[klt_invalid_ids[i]]);
      }
    }

    // Filter matches with RANSAC
    if (prev_multi_frame_.get()) {
      const auto& frame_prev = prev_multi_frame_->frames_[l];
      
      std::vector<uchar> inlier_status 
        = out_rej_.getInliers(tracked_prev, tracked_next, intrinsics.get());
      std::set<size_t> inlier_ids;
      for (int i = 0; i < inlier_status.size(); i++) {
        size_t prev_id = tracked_ids[i];
        if (inlier_status[i]) {
          frame->features_.push_back(tracked_next[i]);
          frame->feature_ids_.push_back(prev_id);
          inlier_ids.insert(prev_id);
        }
      }
      // Remove invalid ids from database
      std::list<size_t> invalid_ids;
      for (const auto& id : frame_prev->feature_ids_) {
        if (inlier_ids.find(id) == inlier_ids.end()) {
          invalid_ids.push_back(id);
        }
      }
      ft_database_.removeFeatures(invalid_ids);
    } 

    // 6. Detect featues again?
    std::vector<cv::Point2f> new_features2 = 
        keypoints::detectFeatures(frame->img_, frame->features_,
            mask,
            detector_params.grid_size_, 
            detector_params.num_features_per_cell_,
            detector_params.min_dist_);  
    for (const auto& ft : new_features2) {
      frame->addFeature(ft, feature_id_);
    }

    // 7. Calculate descriptors for frame
    std::vector<cv::KeyPoint> keypoints_curr2;
    cv::KeyPoint::convert(frame->features_, keypoints_curr2);
    keypoints::describeKeypointsORB(frame->img_, keypoints_curr2, frame->descriptors_);


    // End matchcing for camera


  }

}