#include "calibration/stereo/stereo_params.h"
#include <opencv2/core/persistence.hpp>

namespace stereo{

Params readParams(const std::string& path) {
  
  Params p;

  cv::FileStorage fs(path.c_str(), cv::FileStorage::READ);

  fs["use_stereo"] >> p.use_stereo_;
  fs["keyframe_skip"] >> p.keyframe_skip_;

  cv::FileNode root = fs["stereo_pairs"];
  p.stereo_pairs_.reserve(root.size());
  for (int m=0; m<root.size(); m++) {
    if (root[m].size() == 2) {
      p.stereo_pairs_.emplace_back(root[m][0], root[m][1]);
    }
    else {
      throw std::runtime_error("Stereo pair config invalid");
    }
  }

  std::vector<int> grid_size_vec;
  fs["grid_size"] >> grid_size_vec;
  p.grid_size_ = cv::Size(grid_size_vec[0], grid_size_vec[1]);

  fs["num_features_per_cell"] >> p.num_features_per_cell_;
  fs["min_dist"] >> p.min_dist_;

  fs["sampson_thresh"] >> p.sampson_thresh_;
  fs["distance_ratio"] >> p.distance_ratio_;
  fs["min_inliers"] >> p.min_inliers_;

  return p;
}

} // end namespae stereo