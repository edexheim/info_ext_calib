#ifndef FEATUREMATCHER_H
#define FEATUREMATCHER_H

#include <opencv2/features2d.hpp>

class FeatureMatcher {

  public:

    FeatureMatcher();
    ~FeatureMatcher();

    void initialize();
    void match( const std::vector<cv::KeyPoint>& prev_kp,
                const cv::Mat& prev_desc,
                const std::vector<cv::KeyPoint>& curr_kp,
                const cv::Mat& curr_desc,
                std::vector<cv::Point2f>& prev_pt,
                std::vector<cv::Point2f>& curr_pt);

  private:
    cv::Ptr<cv::DescriptorMatcher> matcher_;

    double dist_thresh_;    
};

#endif