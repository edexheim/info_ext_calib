#ifndef FRAME_H
#define FRAME_H

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

class Frame {

  public:

    Frame(uint8_t cam_id, const cv::Mat& img);
    ~Frame();

    void addFeature(const cv::Point2f& new_pt, size_t& feature_id);
    void addNewFeatures(const std::vector<cv::Point2f>& new_points,
        size_t& feature_id);

  public:

    uint8_t cam_id_;
    const cv::Mat img_;

    std::vector<cv::Point2f> features_;
    std::vector<size_t> feature_ids_;

    cv::Mat descriptors_;

};

#endif