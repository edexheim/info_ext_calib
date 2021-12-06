#ifndef LIN_ALG_UTILS_H
#define LIN_ALG_UTILS_H

#include <Eigen/SparseQR>

Eigen::MatrixXf getLowerMarginalCovariance(
  const Eigen::SparseQR<Eigen::SparseMatrix<float>, Eigen::NaturalOrdering<int> >& qr, 
  int n);

#endif