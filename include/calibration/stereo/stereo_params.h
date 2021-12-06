#pragma once

#include <vector>
#include <opencv2/core/types.hpp>
#include <string>

namespace stereo {

struct Params {
  bool use_stereo_;
  int keyframe_skip_;

  std::vector<std::pair<int, int> > stereo_pairs_;

  cv::Size grid_size_;
  int num_features_per_cell_;
  int min_dist_;

  double sampson_thresh_;
  double distance_ratio_;
  int min_inliers_;
};

Params readParams(const std::string& path);

} // end namespace stereo