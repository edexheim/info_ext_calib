#include "calibration/utils/info_metrics.h"
#include <Eigen/Eigenvalues> 

namespace info_metrics {

double aOpt(const Eigen::MatrixXd& E) {
  return E.trace();
}

double dOpt(const Eigen::MatrixXd& E) {
  return E.determinant();
}

double eOpt(const Eigen::MatrixXd& E) {
  // Is getting the real part sufficient? Or need to invalidate?
  return E.eigenvalues().real().maxCoeff();
}

} // end namespace info_metrics