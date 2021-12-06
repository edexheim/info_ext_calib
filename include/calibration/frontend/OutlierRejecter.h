#ifndef OUTLIERREJECTER_H
#define OUTLIERREJECTER_H

#include <vector>
#include "calibration/frontend/FrontendParams.h"
#include <opencv2/core/types.hpp>
#include "calibration/camera_models/PinholeModel.h"

class OutlierRejecter {

  public:

    OutlierRejecter(const FrontendParams::OutRejParams& params) 
      : params_(params) {;}

    ~OutlierRejecter() {;}

    std::vector<uchar> getInliers(const std::vector<cv::Point2f>& p1,
                                  const std::vector<cv::Point2f>& p2,
                                  const gtsam::PinholeModel* calib) const;

  protected:
    FrontendParams::OutRejParams params_;

};


#endif 