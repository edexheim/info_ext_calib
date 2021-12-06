#include "calibration/data_reader/EurocReader.h"
#include <opencv2/core.hpp>
#include <opencv2/core/persistence.hpp>
#include <fstream>

EurocReader::EurocReader(std::string path)
  : DataReader(path, 2)
{

}

EurocReader::~EurocReader() {

}

bool EurocReader::initialize() {

  // get calibration
  std::string calib0_path = path_ + "cam0/sensor_opencv.yaml";
  cv::FileStorage fs0(calib0_path, cv::FileStorage::READ);
  cv::Mat ext0_mat;
  fs0["T_BS"] >> ext0_mat;
  extrinsics_[0] = gtsam::Pose3(matToEigen(ext0_mat));
  cv::Mat k0;
  fs0["intrinsics"] >> k0;
  intrinsics_[0] = (cv::Mat_<double>(3,3) 
      <<  k0.at<double>(0,0), 0, k0.at<double>(0,2),
          0, k0.at<double>(0,1), k0.at<double>(0,3),
                                        0, 0, 1 );
  fs0["distortion_coefficients"] >> distortions_[0];
  fs0.release();

  std::string calib1_path = path_ + "cam1/sensor_opencv.yaml";
  cv::FileStorage fs1(calib1_path, cv::FileStorage::READ);
  cv::Mat ext1_mat;
  fs1["T_BS"] >> ext1_mat;
  extrinsics_[1] = gtsam::Pose3(matToEigen(ext1_mat));
  cv::Mat k1;
  fs1["intrinsics"] >> k1;
  intrinsics_[1] = (cv::Mat_<double>(3,3) 
      <<  k1.at<double>(0,0), 0, k1.at<double>(0,2),
          0, k1.at<double>(0,1), k1.at<double>(0,3),
                                        0, 0, 1 );
  fs1["distortion_coefficients"] >> distortions_[1];
  fs1.release();


  // get file lists
  std::vector<FileList> img_lists_tmp(num_cam_);
  // Only base files off of first camera (V2_03 has big issues)
  std::string timestamp_path = path_ + "cam0/data.csv";
  std::ifstream timestamp_ifs(timestamp_path);
  if (timestamp_ifs.fail()) {
    return false;
  }
  std::string file_line;
  std::getline(timestamp_ifs, file_line); // first row is text
  while (std::getline(timestamp_ifs, file_line)) {
    std::istringstream iss(file_line);
    std::string elem;
    std::getline(iss, elem, ',');
    double timestamp = std::stod(elem);
    timestamp /= 1e9; // timestamp in naoseconds
    std::string img_name;
    std::getline(iss, img_name, '\r');
    // std::cout << timestamp << " " << img_name << std::endl;
    std::string file_name0 = path_ + "cam0/data/" + img_name;
    std::string file_name1 = path_ + "cam1/data/" + img_name;
    if (std::filesystem::exists(file_name0) && std::filesystem::exists(file_name1)) {
      img_lists_tmp[0].push_back({timestamp, file_name0});
      img_lists_tmp[1].push_back({timestamp, file_name1});
    }
  }

  // get ground truth poses
  std::string gt_pose_path = path_ + "state_groundtruth_estimate0/data.csv";
  std::ifstream gt_pose_ifs(gt_pose_path);
    if (gt_pose_ifs.fail()) {
    return false;
  }

  bool first = true; // images ahead of gt poses
  uint32_t first_valid_img = 0;
  uint32_t img_list_count = 0;
  std::string line;
  std::getline(gt_pose_ifs, line);
  while (std::getline(gt_pose_ifs, line)) {
    std::istringstream iss(line);
    Eigen::Vector3d t;
    Eigen::Quaterniond q;
    std::string elem;
    std::getline(iss, elem, ',');
    double timestamp = std::stod(elem);
    timestamp /= 1e9;
    std::getline(iss, elem, ',');
    t.x() = std::stod(elem);
    std::getline(iss, elem, ',');
    t.y() = std::stod(elem);
    std::getline(iss, elem, ',');
    t.z() = std::stod(elem);
    std::getline(iss, elem, ',');
    q.w() = std::stod(elem);
    std::getline(iss, elem, ',');
    q.x() = std::stod(elem);
    std::getline(iss, elem, ',');
    q.y() = std::stod(elem);
    std::getline(iss, elem, ',');
    q.z() = std::stod(elem);
    gtsam::Rot3 R(q.normalized().toRotationMatrix());
    gtsam::Point3 p(t);
    gtsam::Pose3 pose(R, p);
    // TODO: is interpolation needed for more accuracy?

    // check if ground truth starts after images
    if (first && (timestamp > img_lists_tmp[0][img_list_count].first) ) {
      while (timestamp > img_lists_tmp[0][img_list_count].first) {
        img_list_count++;
      }
      first_valid_img = img_list_count;
      first = false;
    }

    if (timestamp >= img_lists_tmp[0][img_list_count].first) {
      gt_poses_.push_back(pose);
      img_list_count++;
    }

  }

  // re-align 
  for (int l=0; l<num_cam_; l++) {
    img_lists_[l] = FileList( img_lists_tmp[l].begin() + first_valid_img, 
                              img_lists_tmp[l].end() );
  }

  img_sizes_ = getImageSizes();

  return true;
}

std::vector<CameraModelType> EurocReader::getCameraModelType() const {
  return std::vector<CameraModelType>(num_cam_, CameraModelType::PinholeRadtan);
}

std::vector<Eigen::VectorXd> EurocReader::getCalibParams() const {
  std::vector<Eigen::VectorXd> calib_params;
  calib_params.reserve(num_cam_);
  for (int l = 0; l < num_cam_; l++) {
    Eigen::VectorXd params(9);
    params << intrinsics_[l].at<double>(0,0), intrinsics_[l].at<double>(1,1), 
              intrinsics_[l].at<double>(0,1), 
              intrinsics_[l].at<double>(0,2), intrinsics_[l].at<double>(1,2),
              distortions_[l].at<double>(0,0), distortions_[l].at<double>(1,0),
              distortions_[l].at<double>(2,0), distortions_[l].at<double>(3,0);
    calib_params.push_back(params);
  }
  return calib_params;
}