#include "calibration/frontend/keypoints.h"
#include <opencv2/core/fast_math.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/imgproc.hpp>

// For debugging
#include <iostream>

namespace keypoints {

// Derived from Basalt-VIO
// TODO: Put BSD 3-Clause License from keypoints.cpp
std::vector<cv::Point2f> detectFeatures(
    const cv::Mat& img, const std::vector<cv::Point2f>& curr_features,
    const cv::Mat& mask,
    cv::Size grid_size, int num_points_cell, int min_dist) {

  std::vector<cv::Point2f> new_features;

  cv::Mat feature_mask;
  if (mask.empty()) {
    feature_mask = cv::Mat(img.size(), CV_8UC1, cv::Scalar(255));
  }
  else {
    feature_mask = mask.clone();
  }

  const cv::Size patch_size(
      img.cols / grid_size.width, 
      img.rows / grid_size.height);

  const size_t x_start = (img.cols % patch_size.width) / 2;
  const size_t x_stop = img.cols - patch_size.width;

  const size_t y_start = (img.rows % patch_size.height) / 2;
  const size_t y_stop = img.rows - patch_size.height;

  // std::cout << "x_start " << x_start << " x_stop " << x_stop << std::endl;
  // std::cout << "y_start " << y_start << " y_stop " << y_stop << std::endl;

  cv::Mat cells(grid_size, CV_32S, cv::Scalar(0) );

  for (const cv::Point2f& ft : curr_features) {
    if (ft.x >= x_start && ft.y >= y_start) {
      int x = (ft.x - x_start) / patch_size.width;
      int y = (ft.y - y_start) / patch_size.height;
      // if (x < 0 || y < 0 || x >= cell_cols || y >= cell_rows)
        // std::cout << ft.x << " " << ft.y << " " << x << " " << y << std::endl;

      cv::circle(feature_mask, ft, min_dist, cv::Scalar(0), -1);
      if (x >= 0 && y >=0 && x < grid_size.width && y < grid_size.height) {
        cells.at<int>(y, x) += 1;
      }
    }
  }

  // std::cout << "Image size: " << img.rows << " " << img.cols << std::endl;
  // std::cout << "Patch dims: " << patch_size.height << " " << patch_size.width << std::endl;
  // std::cout << "Cell grid dims: " << grid_size.height << " " << grid_size.width << std::endl;

  for (int x = x_start; x <= x_stop; x += patch_size.width) {
    for (int y = y_start; y <= y_stop; y += patch_size.height) {

      // TODO: Add padding if possible to account for FAST size
      int x_left = std::max(0, x-3);
      int y_top = std::max(0, y-3);
      int x_right = x + patch_size.width + 3;
      int y_bottom = y + patch_size.height + 3;
      if (x_right >= img.cols)
        x_right = patch_size.width;
      if (y_bottom >= img.rows)
        y_bottom = patch_size.height;
      int rect_width = y_bottom - y_top;
      int rect_height = x_right - x_left;
      cv::Rect rect(x, y, patch_size.width, patch_size.height);
      cv::Mat sub_img = img(rect);

      int cell_y = (y-y_start)/patch_size.height;
      int cell_x = (x-x_start)/patch_size.width;

      int points_added = 0;
      int threshold = 40;

      while (cells.at<int>(cell_y, cell_x) < num_points_cell && threshold >= 10) {

        std::vector<cv::KeyPoint> detected_points;
        cv::FAST(sub_img, detected_points, threshold);

        std::sort(detected_points.begin(), detected_points.end(),
                  [](const cv::KeyPoint& a, const cv::KeyPoint& b) -> bool {
                    return a.response > b.response;
                  });

        //        std::cout << "Detected " << points.size() << " points.
        //        Threshold "
        //                  << threshold << std::endl;

        for (size_t i = 0; 
            i < detected_points.size() && cells.at<int>(cell_y, cell_x) < num_points_cell;
             i++) {
          // TODO: Should we check for sufficient padding?
          // if (img_raw.InBounds(x + points[i].pt.x, y + points[i].pt.y,
          //                      EDGE_THRESHOLD)) {
          size_t x_global = x + detected_points[i].pt.x;
          size_t y_global = y + detected_points[i].pt.y;
          // if (x_global >= 0 && x_global < img.cols
          //     && y_global >= 0 && y_global < img.rows)

          if (feature_mask.at<uint8_t>(y_global, x_global) > 0) {
            cv::Point2f ft(x_global, y_global);
            cv::circle(feature_mask, ft, min_dist, cv::Scalar(0), -1);
            new_features.emplace_back(ft);
            cells.at<int>(cell_y, cell_x)++;
          }
        }

        threshold /= 2;
      }
    }
  }
  return new_features;
}

std::vector<cv::KeyPoint> detectKeypoints(
    const cv::Mat& img, const std::vector<cv::KeyPoint>& curr_keypoints,
    const cv::Mat& mask,
    cv::Size grid_size, int num_points_cell, int min_dist) {

  std::vector<cv::Point2f> curr_features;
  cv::KeyPoint::convert(curr_keypoints, curr_features);
  std::vector<cv::Point2f> new_features = 
      detectFeatures(img, curr_features, mask, 
          grid_size, num_points_cell, min_dist);
  std::vector<cv::KeyPoint> new_keypoints;
  cv::KeyPoint::convert(new_features, new_keypoints);
  return new_keypoints;
}

void ICAngles(
    const cv::Mat& img, 
    std::vector<cv::KeyPoint>& pts, 
    int halfPatchSize) {

  // pre-compute the end of a row in a circular patch
  std::vector<int> umax(halfPatchSize + 2);

  int v, v0, vmax = cvFloor(halfPatchSize * std::sqrt(2.f) / 2 + 1);
  int vmin = cvCeil(halfPatchSize * std::sqrt(2.f) / 2);
  for (v = 0; v <= vmax; ++v)
    umax[v] = cvRound(std::sqrt((double)halfPatchSize * halfPatchSize - v * v));

  // Make sure we are symmetric
  for (v = halfPatchSize, v0 = 0; v >= vmin; --v)
  {
    while (umax[v0] == umax[v0 + 1])
      ++v0;
    umax[v] = v0;
    ++v0;
  }

  // TODO: Check for boundaries
  // Loop over keypoints and calculate moments for angles
  int step = (int)img.step1();
  size_t ptidx, ptsize = pts.size();
  int rows = img.rows;
  int cols = img.cols;

  for(ptidx = 0; ptidx < ptsize; ptidx++) {
    int cy = cvRound(pts[ptidx].pt.y);
    int cx = cvRound(pts[ptidx].pt.x);

    // Skip orientation calculation if patch near edge
    if ( (cy - halfPatchSize) < 0 || (cx - halfPatchSize) < 0 
        || (cy + halfPatchSize) >= rows || (cx + halfPatchSize) > cols) {
      // TODO: Set angle to 0?
      continue;
    }

    const uchar* center = &img.at<uchar>(cy, cx);

    int m_01 = 0, m_10 = 0;

    // Treat the center line differently, v=0
    for (int u = -halfPatchSize; u <= halfPatchSize; ++u)
      m_10 += u * center[u];

    // Go line by line in the circular patch
    for (int v = 1; v <= halfPatchSize; ++v) {
      // Proceed over the two lines
      int v_sum = 0;
      int d = umax[v];
      for (int u = -d; u <= d; ++u) {
        int val_plus = center[u + v*step], val_minus = center[u - v*step];
        v_sum += (val_plus - val_minus);
        m_10 += u * (val_plus + val_minus);
      }
      m_01 += v * v_sum;
    }

    pts[ptidx].angle = cv::fastAtan2((float)m_01, (float)m_10);
    pts[ptidx].size = 2*halfPatchSize + 1;
  }

}

std::vector<cv::KeyPoint> detectKeypointsWithAngle(
    const cv::Mat& img, const std::vector<cv::KeyPoint>& curr_keypoints,
    const cv::Mat& mask,
    cv::Size grid_size, int num_points_cell, int min_dist) {

  std::vector<cv::KeyPoint> new_keypoints;
  new_keypoints = detectKeypoints(img, curr_keypoints, mask, 
      grid_size, num_points_cell, min_dist);
  int half_patch_size = 15;
  ICAngles(img, new_keypoints, half_patch_size);
  return new_keypoints;
}

void describeKeypointsBrief(
    const cv::Mat& img, 
    std::vector<cv::KeyPoint>& keypoints,
    cv::Mat& descriptors,
    bool use_angle) {

  cv::Ptr<cv::Feature2D> brief_extractor = cv::xfeatures2d::BriefDescriptorExtractor::create(32, use_angle);
  brief_extractor->compute(img, keypoints, descriptors);
}

void describeKeypointsORB(
    const cv::Mat& img, 
    std::vector<cv::KeyPoint>& keypoints,
    cv::Mat& descriptors) {

  // NOTE: ORB compute will remove keypoints close to border
  // Default edge threshold is 31 (Why is it the same as patch size?)
  int num_features = 0;
  double scale_factor = 1.0f;
  int num_levels = 1;
  int edge_threshold = 0; // Does this cause any out of bounds issues?
  int first_level = 0;
  int WTA_K = 2;
  int patch_size = 31;
  int fast_threshold = 20;
  cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create(
      num_features, scale_factor, num_levels, edge_threshold, 
      first_level, WTA_K, cv::ORB::HARRIS_SCORE, patch_size, fast_threshold);
  detector->compute(img, keypoints, descriptors);
}

void runByImageBorder(std::vector<cv::KeyPoint>& keypoints, 
    std::vector<size_t> valid_indices,
    cv::Size image_size, int border_size) {
  // if( border_size > 0) {
  //   if (image_size.height <= border_size * 2 || image_size.width <= border_size * 2) {
  //     keypoints.clear();
  //   }
  //   else {
  //     for (size_t i = 0; i < keypoints.size(); i++) {

  //     }
  //     keypoints.erase( std::remove_if(keypoints.begin(), keypoints.end(),
  //         cv::RoiPredicate(cv::Rect(cv::Point(border_size, border_size),
  //             cv::Point(image_size.width - border_size, image_size.height - border_size)))),
  //                 keypoints.end() );
  //   }
  // }
}

} // end namespace stereo