#pragma once

#include <vector>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <opencv2/core/mat.hpp>

namespace stereo {

void getSampsonMask(
    const std::vector<gtsam::Point3>& rays1,
    const std::vector<gtsam::Point3>& rays2,
    const gtsam::Pose3& T_c1_c2,
    cv::Mat& epipolar_mask, 
    double sampson_thresh);

void getSampsonMasks(
    const std::vector<std::pair<int,int> >& match_pairs,
    const std::vector<gtsam::Pose3>& curr_extrinsics_estimates,
    const std::vector<std::vector<gtsam::Point3> >& rays,
    std::vector<cv::Mat>& epipolar_masks, 
    double sampson_thresh);

void matchFeatures(
    const cv::Mat& descs1, const cv::Mat& desc2,
    const cv::Mat& epipolar_mask,
    std::vector<cv::DMatch>& matches,
    double distance_ratio);

void matchFeatures(
    const std::vector<std::pair<int,int> >& match_pairs,
    const std::vector<cv::Mat>& descs,
    const std::vector<cv::Mat>& epipolar_masks,
    std::vector<std::vector<cv::DMatch> >& matches,
    double distance_ratio);

void pruneMatchesRansac(
    const std::vector<std::pair<int,int> >& match_pairs,
    const std::vector<std::vector<gtsam::Point3> >& rays,
    const std::vector<std::vector<cv::DMatch> >& initial_matches,
    const std::vector<gtsam::Pose3>& curr_extrinsics_estimates,
    std::vector<std::vector<cv::DMatch> >& pruned_matches,
    int min_inliers);

} // end namespace stereo