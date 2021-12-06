#include "calibration/frontend/keyframe_selection/KeyframeSelecter.h"
#include "calibration/utils/gtsam_utils.h"

KeyframeSelecter::KeyframeSelecter(const FrontendParams::KeyframeSelecterParams& params)
  : params_(params)
{

};

KeyframeSelecter::~KeyframeSelecter() {

};