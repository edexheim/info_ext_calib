#include "calibration/stereo/stereo_matching.h"

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include "calibration/factors/epipolar/EpipolarFactor.h"

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

namespace stereo {

void getSampsonMask(
    const std::vector<gtsam::Point3>& rays1,
    const std::vector<gtsam::Point3>& rays2,
    const gtsam::Pose3& T_c1_c2,
    cv::Mat& epipolar_mask, 
    double sampson_thresh) {

  int num1 = rays1.size();
  int num2 = rays2.size();

  epipolar_mask = cv::Mat(num1, num2, CV_8UC1);

  double sampson_thresh_sq = sampson_thresh * sampson_thresh;

  // Prior epipolar matrix guess for loose pruning
  gtsam::Matrix3 E = gtsam::skewSymmetric(T_c1_c2.translation()) * T_c1_c2.rotation().matrix();
  for (int i=0; i<num1; i++) {
    uchar* const mask_ptr = epipolar_mask.ptr<uchar>(i);
    for (int j=0; j<num2; j++) {
      double error_sq = gtsam::EpipolarFactor::SampsonErrorSq(E, rays1[i], rays2[j]);
      mask_ptr[j] = (error_sq < sampson_thresh_sq);
    }
  }

}


void getSampsonMasks(
    const std::vector<std::pair<int,int> >& match_pairs,
    const std::vector<gtsam::Pose3>& extrinsics_estimates,
    const std::vector<std::vector<gtsam::Point3> >& rays,
    std::vector<cv::Mat>& epipolar_masks, 
    double sampson_thresh) {

  int NUM_PAIRS = match_pairs.size();

  // Allocate mask vector
  epipolar_masks.clear();
  epipolar_masks.resize(NUM_PAIRS);

  tbb::parallel_for(tbb::blocked_range<size_t>(0, NUM_PAIRS),
      [&](const tbb::blocked_range<size_t>& range_m) {
  for (size_t m = range_m.begin(); m != range_m.end(); m++) {
    int l1 = match_pairs[m].first;
    int l2 = match_pairs[m].second;
    // Prior epipolar matrix guess for loose pruning
    const gtsam::Pose3& T_c1 = extrinsics_estimates[l1];
    const gtsam::Pose3& T_c2 = extrinsics_estimates[l2];
    gtsam::Pose3 T_c1_c2 = T_c1.between(T_c2);
    
    getSampsonMask(rays[l1], rays[l2], T_c1_c2, epipolar_masks[m], sampson_thresh);
  } });
}

void matchFeatures(
    const cv::Mat& descs1, const cv::Mat& descs2,
    const cv::Mat& epipolar_mask,
    std::vector<cv::DMatch>& matches,
    double distance_ratio) {

  int num1 = descs1.rows;
  int num2 = descs2.rows;

  // Calculate distances of 2NN
  // Note: These functions parallelize internally
  cv::Mat dist1, nidx1, dist2, nidx2;
  cv::batchDistance(descs1, descs2, 
      dist1, CV_32S, nidx1,
      cv::NORM_HAMMING, 2, epipolar_mask);
  cv::batchDistance(descs2, descs1, 
      dist2, CV_32S, nidx2,
      cv::NORM_HAMMING, 2, epipolar_mask.t()); 

  // Consistency and distance ratio checks
  for (int i=0; i<num1; i++) {
    int j_match = nidx1.at<int>(i,0);
    if (j_match >= 0) {
      // Cross check
      if (i == nidx2.at<int>(j_match,0)) {
        // Distance ratio
        if ( (dist1.at<int>(i,0) <= distance_ratio*dist1.at<int>(i,1))
            && (dist2.at<int>(j_match,0) <= distance_ratio*dist2.at<int>(j_match,1)) ) {

            // std::cout << dist1.at<int>(i,0) << " " << dist1.at<int>(i,1) << std::endl;
            // std::cout << i << " " << nidx2.at<int>(j_match,0) << " " << nidx2.at<int>(j_match,1) << std::endl;
            // std::cout << j_match << " " << nidx1.at<int>(i,0) << " " << nidx1.at<int>(i,1) << std::endl;
            cv::DMatch dmatch(i, j_match, dist1.at<int>(i,0));
            matches.push_back(dmatch);
        }
      }
    }
  }
}

void matchFeatures(
    const std::vector<std::pair<int,int> >& match_pairs,
    const std::vector<cv::Mat>& descs,
    const std::vector<cv::Mat>& epipolar_masks,
    std::vector<std::vector<cv::DMatch> >& matches,
    double distance_ratio) {

  int NUM_PAIRS = match_pairs.size();
  matches.clear();
  matches.resize(NUM_PAIRS);

  for (int m = 0; m < NUM_PAIRS; m++) {
    int l1 = match_pairs[m].first;
    int l2 = match_pairs[m].second;
    
    matchFeatures(descs[l1], descs[l2], 
        epipolar_masks[m], matches[m], distance_ratio);
  }

}

void pruneMatchesRansac(
    const std::vector<std::pair<int,int> >& match_pairs,
    const std::vector<std::vector<gtsam::Point3> >& rays,
    const std::vector<std::vector<cv::DMatch> >& initial_matches,
    const std::vector<gtsam::Pose3>& curr_extrinsics_estimates,
    std::vector<std::vector<cv::DMatch> >& pruned_matches,
    int min_inliers) {

  int NUM_PAIRS = match_pairs.size();
  pruned_matches.clear();
  pruned_matches.resize(NUM_PAIRS);

  tbb::parallel_for(tbb::blocked_range<size_t>(0, NUM_PAIRS),
      [&](const tbb::blocked_range<size_t>& range_m) {
  for (size_t m = range_m.begin(); m != range_m.end(); m++) {
  // for (int m = 0; m < NUM_PAIRS; m++) {
    if (initial_matches[m].size() >= min_inliers) {
      int l1 = match_pairs[m].first;
      int l2 = match_pairs[m].second;
      std::vector<cv::Point2f> points_i, points_j;
      points_i.reserve(initial_matches[m].size());
      points_j.reserve(initial_matches[m].size());
      for (int i=0; i<initial_matches[m].size(); i++) {
        gtsam::Point3 p_i = rays[l1][initial_matches[m][i].queryIdx];
        gtsam::Point3 p_j = rays[l2][initial_matches[m][i].trainIdx];
        points_i.emplace_back(p_i.x(), p_i.y());
        points_j.emplace_back(p_j.x(), p_j.y());
      }
      std::vector<uchar> status;
      cv::Mat E = cv::findEssentialMat( points_i, points_j, cv::Mat::eye(3,3,CV_32F),
                                        cv::RANSAC,  
                                        0.999 /*ransac_prob*/, 
                                        1e-3 /*ransac_thresh*/,
                                        status);
      

      // Use extrinsic guess for triangulation
      // More trustworthy than output of 5-pt estimation?
      const gtsam::Pose3& T_c1 = curr_extrinsics_estimates[l1];
      const gtsam::Pose3& T_c2 = curr_extrinsics_estimates[l2];
      gtsam::Pose3 T_c2_c1 = T_c2.between(T_c1);
      Eigen::Matrix3f R_eigen = T_c2_c1.rotation().matrix().cast<float>();
      Eigen::Vector3f t_eigen = T_c2_c1.translation().cast<float>();

      cv::Mat R, t;
      cv::eigen2cv(R_eigen, R);
      cv::eigen2cv(t_eigen, t);
      cv::Mat P0 = cv::Mat::eye(3, 4, CV_32F);
      cv::Mat P1(3, 4, CV_32F);
      R.copyTo(P1(cv::Rect(0,0,3,3)));
      t.copyTo(P1(cv::Rect(3,0,1,3)));

      cv::Mat triangulated_pts_h;
      cv::triangulatePoints(P0, P1, points_i, points_j, triangulated_pts_h);
      cv::transpose(triangulated_pts_h, triangulated_pts_h);
      std::vector<cv::Point3f> triangulated_pts;
      cv::convertPointsFromHomogeneous(triangulated_pts_h, triangulated_pts);
       
      int count = 0;
      for (int i=0; i<triangulated_pts.size(); i++) {
        if (status[i]) {
          // Check if in front of camera and not far (50 is OpenCV default)
          if (triangulated_pts[i].z > 0 && triangulated_pts[i].z < 50) {
            count++;
          }
          else {
            status[i] = 0;
          }
        }
      }

      // int count = 0;
      // for (int i=0; i<status.size(); i++) {
      //   if (status[i])
      //     count++;
      // }
      // std::cout << "Pair: " << m << " stereo inliers: " << count << std::endl;
      
      if (count >= min_inliers) {
        for (int i=0; i<status.size(); i++) {
          if (status[i]) {
            pruned_matches[m].push_back(initial_matches[m][i]);
            // gtsam::Point3 p(
            //     triangulated_pts[i].x,
            //     triangulated_pts[i].y,
            //     triangulated_pts[i].z);
            // Transform point to vehicle frame
            // points_3d[m].emplace_back(T_c1*p);
          }
        }
      }
    }
  // }
  } });

}


} // end namespace stereo