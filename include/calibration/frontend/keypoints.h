#pragma once

#include <opencv2/core/mat.hpp>
#include <vector>

namespace keypoints {

// Derived from Basalt-VIO
// TODO: Put BSD 3-Clause License from keypoints.cpp
std::vector<cv::Point2f> detectFeatures(
    const cv::Mat& img, const std::vector<cv::Point2f>& curr_features,
    const cv::Mat& mask,
    cv::Size grid_size, int num_points_cell, int min_dist);

std::vector<cv::KeyPoint> detectKeypoints(
    const cv::Mat& img, const std::vector<cv::KeyPoint>& curr_keypoints,
    const cv::Mat& mask,
    cv::Size grid_size, int num_points_cell, int min_dist);

// Modified from OpenCV version
void ICAngles(
    const cv::Mat& img, 
    std::vector<cv::KeyPoint>& pts, 
    int halfPatchSize);

std::vector<cv::KeyPoint> detectKeypointsWithAngle(
    const cv::Mat& img, const std::vector<cv::KeyPoint>& curr_keypoints,
    const cv::Mat& mask,
    cv::Size grid_size, int num_points_cell, int min_dist);

void describeKeypointsBrief(
    const cv::Mat& img, 
    std::vector<cv::KeyPoint>& keypoints,
    cv::Mat& descriptors,
    bool use_angle);

void describeKeypointsORB(
    const cv::Mat& img, 
    std::vector<cv::KeyPoint>& keypoints,
    cv::Mat& descriptors);

// Modified from OpenCV version
void runByImageBorder(std::vector<cv::KeyPoint>& keypoints, 
    cv::Size image_size, int border_size);

} // end namespace stereo