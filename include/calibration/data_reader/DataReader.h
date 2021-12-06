#ifndef DATAREADER_H
#define DATAREADER_H

#include <string>
#include <vector>
#include "calibration/utils/basic_types.h"
#include "calibration/frontend/Frame.h"
#include <Eigen/Dense>
#include <gtsam/geometry/Pose3.h>

typedef std::vector<std::pair<Timestamp, std::string> > FileList;

class DataReader {

  public:

    typedef std::unique_ptr<DataReader> unique_ptr;

    DataReader(std::string path, uint8_t num_cam);
    virtual ~DataReader();

    // Virtual functions
    virtual bool initialize() = 0;
    virtual std::vector<CameraModelType> getCameraModelType() const = 0;
    virtual std::vector<Eigen::VectorXd> getCalibParams() const = 0;
    virtual std::vector<std::shared_ptr<Frame> > getFrames(size_t frame_id) const;
    virtual std::vector<cv::Size> getImageSizes() const;
    virtual std::vector<cv::Mat> getMasks(size_t frame_id) const;
    virtual size_t getBaseCam() const {return 0;}

    // Accessors
    uint8_t getNumCam() const {return num_cam_;}
    size_t getNumFrames() const {return img_lists_[0].size();}
    Timestamp getTimestamp(size_t frame_id) const {return img_lists_[0][frame_id].first;}
    cv::Mat getIntrinsics(uint8_t cam_id) const {return intrinsics_[cam_id];}
    cv::Mat getDistortionParams(uint8_t cam_id) const {return distortions_[cam_id];}
    std::vector<gtsam::Pose3> getExtrinsics() const {return extrinsics_;};
    gtsam::Pose3 getExtrinsics(uint8_t cam_id) const {return extrinsics_[cam_id];}
    gtsam::Pose3 getGroundTruthPose(size_t frame_id) const {return gt_poses_[frame_id];}
    gtsam::Pose3 getGroundTruthOdom(size_t frame_id) const;

  protected:
    const std::string path_;
    const uint8_t num_cam_;

    std::vector<FileList> img_lists_;
    std::vector<gtsam::Pose3> gt_poses_;
    std::vector<cv::Mat> intrinsics_;
    std::vector<cv::Mat> distortions_;
    std::vector<gtsam::Pose3> extrinsics_;
    std::vector<cv::Size> img_sizes_;

    static Eigen::Matrix4d matToEigen(const cv::Mat& Rt);
};


#endif 