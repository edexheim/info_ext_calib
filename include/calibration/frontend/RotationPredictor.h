#pragma once

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <opencv2/core/types.hpp>
#include "calibration/camera_models/PinholeModel.h"

template<class CALIB>
class RotationPredictor {

  public:

    RotationPredictor(const CALIB* calib, const gtsam::Rot3 R_c2_c1,
        const cv::Size& img_size)
        : calib_(calib), R_c2_c1_(R_c2_c1), img_size_(img_size) {}

    std::vector<cv::Point2f> predict(const std::vector<cv::Point2f>& pts_c1) const {
      std::vector<cv::Point2f> pts_c2;
      // Get angle in radians
      std::pair<gtsam::Unit3, double> ax_ang = R_c2_c1_.axisAngle();
      static const double angle_thresh = (1.0/180.0) * 3.14;
      if (ax_ang.second < angle_thresh) {
        // TODO: Avoid copy?
        pts_c2 = pts_c1;
      }
      else {
        pts_c2.reserve(pts_c1.size());
        for (const auto& pt1 : pts_c1) {
          gtsam::Point2 pt1_u = calib_->calibrate(gtsam::Point2(pt1.x, pt1.y));
          gtsam::Point3 ray1(pt1_u.x(), pt1_u.y(), 1.0);
          gtsam::Point3 ray2 = R_c2_c1_*ray1;
          if (ray2.z() > 0) {
            gtsam::Point2 pt2_u(ray2.x()/ray2.z(), ray2.y()/ray2.z());
            gtsam::Point2 pt2 = calib_->uncalibratePixelDeriv(pt2_u);
            cv::Point2f pt_pred(pt2.x(), pt2.y());
            if (pt_pred.x >= 0 && pt_pred.y >= 0 
                && pt_pred.x < img_size_.width && pt_pred.y < img_size_.height) {
              pts_c2.push_back(pt_pred);
            } 
            else {
              pts_c2.push_back(pt1);
            }
          }
          else {
            pts_c2.push_back(pt1);
          }
        }
      }

      return pts_c2;
    }

  private:

    const CALIB* const calib_;
    gtsam::Rot3 R_c2_c1_;
    cv::Size img_size_;

};
