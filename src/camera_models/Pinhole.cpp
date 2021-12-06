#include "calibration/camera_models/Pinhole.h"

namespace gtsam {

/* ************************************************************************* */
Vector5 Pinhole::vector() const {
  Vector5 v;
  v << fx_, fy_, s_, u0_, v0_;
  return v;
}

/* ************************************************************************* */
Point2 Pinhole::calibrate(const Point2& p) const {
  const double u = p.x(), v = p.y();
  double delta_u = u - u0_, delta_v = v - v0_;
  double inv_fx = 1 / fx_, inv_fy = 1 / fy_;
  double inv_fy_delta_v = inv_fy * delta_v;
  double inv_fx_s_inv_fy = inv_fx * s_ * inv_fy;

  Point2 point(inv_fx * (delta_u - s_ * inv_fy_delta_v), inv_fy_delta_v);
  return point;
}

/* ************************************************************************* */
Point2 Pinhole::uncalibrate(
    const Point2& p, OptionalJacobian<2, Pinhole::DIM_> Dcal,
    OptionalJacobian<2, 2> Dp) const {

  const double x = p.x(), y = p.y();
  if (Dcal) *Dcal << x, 0.0, y, 1.0, 0.0, 0.0, y, 0.0, 0.0, 1.0;
  if (Dp) *Dp << fx_, s_, 0.0, fy_;
  return Point2(fx_ * x + s_ * y + u0_, fy_ * y + v0_);
}


/* ************************************************************************* */
void Pinhole::print(const std::string& s) const { 
  this->PinholeModel::print(s); 
}

/* ************************************************************************* */
bool Pinhole::equals(const Pinhole& K, double tol) const {
  return this->PinholeModel::equals(K, tol);
}

/* ************************************************************************* */
Pinhole Pinhole::retract(const Vector& d) const {
  return Pinhole(vector() + d);
}

/* ************************************************************************* */
Vector Pinhole::localCoordinates(const Pinhole& T2) const {
  return T2.vector() - vector();
}

} // end namespace gtsam