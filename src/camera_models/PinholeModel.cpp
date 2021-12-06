#include "calibration/camera_models/PinholeModel.h"

namespace gtsam {

/* ************************************************************************* */
Matrix3 PinholeModel::K() const {
  Matrix3 K;
  K << fx_, s_, u0_, 0.0, fy_, v0_, 0.0, 0.0, 1.0;
  return K;
}

/* ************************************************************************* */
void PinholeModel::print(const std::string& s) const { 
  gtsam::print((Matrix)K(), s); 
}

/* ************************************************************************* */
bool PinholeModel::equals(const PinholeModel& K, double tol) const {
  return (std::fabs(fx_ - K.fx_) < tol && std::fabs(fy_ - K.fy_) < tol &&
          std::fabs(s_ - K.s_) &&
          std::fabs(u0_ - K.u0_) < tol && std::fabs(v0_ - K.v0_) < tol);
}

} // end namespace gtsam