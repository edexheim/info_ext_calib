#include "calibration/data_reader/DataReader.h"
#include <opencv2/imgcodecs.hpp>

DataReader::DataReader(std::string path, uint8_t num_cam)
    : path_(path), num_cam_(num_cam),
      img_lists_(num_cam), intrinsics_(num_cam), 
      extrinsics_(num_cam), distortions_(num_cam) {

}

DataReader::~DataReader() {

}

std::vector<std::shared_ptr<Frame> > DataReader::getFrames(size_t frame_id) const {
  std::vector<std::shared_ptr<Frame> > frames;
  frames.reserve(num_cam_);
  for (int l=0; l<num_cam_; l++) {
    // std::cout << l << " " << img_lists_[l][frame_id].first << " " << img_lists_[l][frame_id].second << std::endl;
    frames.emplace_back(
      std::make_shared<Frame>(
        l, 
        cv::imread(img_lists_[l][frame_id].second, cv::IMREAD_GRAYSCALE) 
      ) 
    );
  }
  return frames;
}

std::vector<cv::Size> DataReader::getImageSizes() const {
  std::vector<cv::Size> img_sizes;
  img_sizes.reserve(num_cam_);
  for (int l=0; l<num_cam_; l++) {
    cv::Mat mat = cv::imread(img_lists_[l][0].second, cv::IMREAD_GRAYSCALE);
    img_sizes.push_back(mat.size());
  }
  return img_sizes;
}

std::vector<cv::Mat> DataReader::getMasks(size_t frame_id) const {
  // TODO: Return empty by default and perform checks elsewhere
  std::vector<cv::Mat> masks;
  masks.reserve(num_cam_);
  for (int l=0; l<num_cam_; l++) {
    masks.emplace_back(cv::Mat(img_sizes_[l], CV_8UC1, cv::Scalar(255)));
  }
  return masks;
}

gtsam::Pose3 DataReader::getGroundTruthOdom(size_t frame_id) const {
  gtsam::Pose3 odom;
  if (frame_id == 0) {
    // odom = gtsam::Pose3::identity();
    odom = gt_poses_[frame_id];
  }
  else {
    odom = gt_poses_[frame_id-1].inverse() * gt_poses_[frame_id];
  }
  return odom;
}

Eigen::Matrix4d DataReader::matToEigen(const cv::Mat& Rt) {
 Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  for (int i=0; i<3; i++) {
    for (int j=0; j<4; j++) {
      T(i,j) = Rt.at<double>(i,j);
    }
  }
  return T;
}