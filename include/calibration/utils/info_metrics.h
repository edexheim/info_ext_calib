#ifndef INFO_METRICS_H
#define INFO_METRICS_H

#include <Eigen/Core>

namespace info_metrics {

double aOpt(const Eigen::MatrixXd& E);

double dOpt(const Eigen::MatrixXd& E);

double eOpt(const Eigen::MatrixXd& E);

} // end namespace info_metrics

#endif