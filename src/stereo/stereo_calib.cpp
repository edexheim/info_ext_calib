#include "calibration/stereo/stereo_calib.h"

#include "calibration/frontend/keypoints.h"
#include "calibration/stereo/stereo_matching.h"

#include <unordered_set>

// For visualization of matches
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>

namespace stereo {

void setupCalib(
    const std::vector<gtsam::PinholeModel::shared_ptr>& intrinsics,
    const std::vector<gtsam::Pose3>& extrinsics_estimate,
    const std::vector<std::shared_ptr<Frame> >& frames,
    std::vector<std::pair<size_t, size_t> >& landmark_landmark_matches,
    std::vector<std::pair<size_t, StereoMatch> >& landmark_point_matches,
    std::vector<std::pair<StereoMatch, StereoMatch> >& point_point_matches,
    const Params& stereo_params) {

  // gttic_(stereo_setupCalib);

  const int NUM_CAM = intrinsics.size();
  landmark_landmark_matches.clear();
  landmark_point_matches.clear();
  point_point_matches.clear();

  const int NUM_PAIRS = stereo_params.stereo_pairs_.size();

  // Get features and descriptors
  // gttic_(FeatureDetection);
  std::vector<std::vector<cv::KeyPoint> > features(NUM_CAM);
  std::vector<std::vector<gtsam::Point3> > rays(NUM_CAM);
  std::vector<cv::Mat> descs(NUM_CAM);
  for (int l = 0; l < NUM_CAM; l++) {
    // Detect and describe features
    std::vector<cv::KeyPoint> existing_features;
    cv::KeyPoint::convert(frames[l]->features_, existing_features);
    features[l] = keypoints::detectKeypointsWithAngle(frames[l]->img_, 
        existing_features, cv::Mat(),
        stereo_params.grid_size_, 
        stereo_params.num_features_per_cell_,
        stereo_params.min_dist_);
    // Append existing features
    features[l].insert(
        features[l].end(), 
        existing_features.begin(), 
        existing_features.end() );

    // Need to remove keypoints near border first, because otherwise
    //   ORB will remove them without notifying which
    keypoints::describeKeypointsORB(frames[l]->img_, features[l], descs[l]);

    // Undistort features
    features[l].reserve(features[l].size());
    for (int j = 0; j<features[l].size(); j++) {
      gtsam::Point2 p_u = intrinsics[l]->calibrate(gtsam::Point2(features[l][j].pt.x, features[l][j].pt.y));
      gtsam::Point3 r(p_u.x(), p_u.y(), 1.0);  
      rays[l].push_back(r);
    }
  }
  // gttoc_(FeatureDetection);

  // gttic_(SampsonDistance);
  std::vector<cv::Mat> epipolar_masks;
  stereo::getSampsonMasks(stereo_params.stereo_pairs_, extrinsics_estimate, rays, 
      epipolar_masks, stereo_params.sampson_thresh_);
  // gttoc_(SampsonDistance);

  // gttic_(FeatureMatching);
  // gttic_(DescriptorMatching);
  std::vector<std::vector<cv::DMatch> > descriptor_matches;
  stereo::matchFeatures(stereo_params.stereo_pairs_, descs, epipolar_masks, 
      descriptor_matches, stereo_params.distance_ratio_);
  // gttoc_(DescriptorMatching);

  // gttic_(Ransac);
  std::vector<std::vector<cv::DMatch> > pairwise_matches;
  stereo::pruneMatchesRansac(stereo_params.stereo_pairs_, rays, 
      descriptor_matches, extrinsics_estimate, 
      pairwise_matches,
      stereo_params.min_inliers_);
  // gttoc_(Ransac);
  // gttoc_(FeatureMatching);

  // Get unique features and assign IDs
  std::vector<size_t> num_fts(NUM_CAM);
  for (int l = 0; l < NUM_CAM; l++) {
    num_fts[l] = frames[l]->features_.size();
  }
  for (int m = 0; m < NUM_PAIRS; m++) {
    int l1 = stereo_params.stereo_pairs_[m].first;
    int l2 = stereo_params.stereo_pairs_[m].second;
    // Check if feature did not already exist in frame
    size_t max_ind1 = features[l1].size() - num_fts[l1];
    size_t max_ind2 = features[l2].size() - num_fts[l2];

    for (int i = 0; i < pairwise_matches[m].size(); i++) {
      const cv::DMatch& match = pairwise_matches[m][i];
      int ind1 = match.queryIdx;
      int ind2 = match.trainIdx;
      const cv::Point2f& pt1 = features[l1][ind1].pt;
      const cv::Point2f& pt2 = features[l2][ind2].pt;

      // Determine existing features
      bool ft1_exists = false;
      size_t ft1_id = 0;
      if (ind1 >= max_ind1) {
        ft1_exists = true;
        ft1_id = frames[l1]->feature_ids_[ind1-max_ind1];
      }

      bool ft2_exists = false;
      size_t ft2_id = 0;
      if (ind2 >= max_ind2) {
        ft2_exists = true;
        ft2_id = frames[l2]->feature_ids_[ind2-max_ind2];
      }

      if (ft1_exists && ft2_exists) {
        landmark_landmark_matches.push_back({ft1_id, ft2_id});
      }
      else if (ft1_exists && !ft2_exists) {
        landmark_point_matches.push_back({ft1_id, StereoMatch(pt2,l2)});
      }
      else if (!ft1_exists && ft2_exists) {
        landmark_point_matches.push_back({ft2_id, StereoMatch(pt1,l1)});
      }
      else {
        point_point_matches.push_back({StereoMatch(pt1,l1), StereoMatch(pt2,l2)});
      }

    }

  }
      
  // // Matches visualization
  // for (int m = 0; m < NUM_PAIRS; m++) {
  //   int l1 = stereo_params.stereo_pairs_[m].first;
  //   int l2 = stereo_params.stereo_pairs_[m].second;
  //   cv::Mat matches_img;
  //   cv::drawMatches(frames[l1]->img_, features[l1],
  //       frames[l2]->img_, features[l2],
  //       pairwise_matches[m], matches_img);
  //   // for (const auto& match : pairwise_matches[m]) {
  //   //   std::cout << match.queryIdx << " " << match.trainIdx << std::endl;
  //   //   std::cout << features[l1][match.queryIdx].pt << " " << features[l2][match.trainIdx].pt << std::endl;
  //   // }
  //   // std::cout << pairwise_matches[m].size() << " " << descriptor_matches[m].size() << " " << features[l1].size() << " " << features[l2].size() << std::endl;
  //   cv::imshow("Matches", matches_img);
  //   cv::waitKey();
  // }


  // gttoc_(stereo_setupCalib);

}

} // end namespace stereo