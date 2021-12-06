#include "calibration/frontend/Frame.h"

Frame::Frame(uint8_t cam_id, const cv::Mat& img)
  : cam_id_(cam_id), img_(img) {

}

Frame::~Frame() {

}

void Frame::addFeature(const cv::Point2f& new_pt, size_t& feature_id) {
  features_.push_back(new_pt);
  feature_ids_.push_back(feature_id); 
  feature_id++;
}

void Frame::addNewFeatures(const std::vector<cv::Point2f>& new_points,
    size_t& feature_id) {

  for (const auto& pt : new_points) {
    this->addFeature(pt, feature_id);
  }
}