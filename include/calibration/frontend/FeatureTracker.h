#ifndef FEATURETRACKER_H
#define FEATURETRACKER_H

#include "calibration/frontend/FrontendParams.h"
#include <list>
#include "calibration/frontend/Frame.h"
#include "calibration/frontend/OutlierRejecter.h"
#include "calibration/camera_models/PinholeModel.h"
#include <gtsam/geometry/Pose3.h>

class FeatureTracker {

  public:

    FeatureTracker(const FrontendParams::TrackerParams& params);
    ~FeatureTracker();

    std::list<size_t> track(const Frame* frame_prev, 
                            Frame* frame_next, 
                            const gtsam::PinholeModel* calib,
                            const OutlierRejecter& out_rej,
                            const gtsam::Pose3& T_c2_c1) const;

  private:
    const FrontendParams::TrackerParams params_;

};

#endif