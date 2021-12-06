#include "calibration/utils/lin_alg_utils.h"
#include <Eigen/Core>

#include <iostream>

// Solving R^T R E = I for bottom right of E (covariance)
// TODO: Could probably be more efficient (zero rows not useful)
// TODO: templating
Eigen::MatrixXf getLowerMarginalCovariance(
  const Eigen::SparseQR<Eigen::SparseMatrix<float>, Eigen::NaturalOrdering<int> >& qr, 
  int n) {

  // Basic QR solution used:
  // RRQR gives R = [R11 R12   = [R11 0]
  //                 R21 R22]  = [0   0]
  // R22 is k x k
  int k = qr.cols() - qr.rank();

  // extrinsics at end of original Jacobian
  // need to monitor indices of permutation
  int ext_ind = qr.cols() - n;

  std::cout << qr.cols() << " " << qr.rank() << " " << n << std::endl;

  // find index of permutation we need to calculate up to
  Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic> perm = qr.colsPermutation();
  int ind = qr.cols()-1;
  while (perm.indices()(ind) > ext_ind) {
    ind--;
  }
  std::cout << ind << std::endl;
  // num of columns we need to look at
  int m = qr.cols() - ind;
  // permutation to obtain original ordering
  Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic> 
    perm_E((perm.indices().bottomRows(m).array()-ind).matrix());

  // column-major upper triangular view of R
  Eigen::SparseMatrix<float> R
    = qr.matrixR().block(ind, ind, m, m).template triangularView<Eigen::Upper>();


  // in Basic QR, bottom columns and rows of covariance are zeroed out
  Eigen::MatrixXf E_marg = Eigen::MatrixXf::Zero(m,m);
  
  // precaclulate inverse diagonal terms
  std::vector<float> diag_inv(m, 0.0);
  for (int i=0; i<m-k; i++) {
    diag_inv[i] = 1.0 / R.coeff(i, i);
  }
  
  // TODO: Speedup by exploiting sparsity of R?
  for (int l=m-k-1; l>=0; l--) {
    float sum_j = 0;
    // basic solution ignores rightmost k columns
    for (int j=l+1; j<m-k; j++) {
      sum_j += R.coeff(l,j) * E_marg(j,l);
    }
    E_marg(l,l) = diag_inv[l] * (diag_inv[l] - sum_j);

    for (int i=l-1; i>=0; i--) {
      float sum_j = 0;
      // basic solution ignores rightmost k columns
      for (int j=i+1; j<m-k; j++) {
        sum_j += R.coeff(i,j) * E_marg(j,l);
      }
      E_marg(i,l) = diag_inv[i] * -sum_j;
      E_marg(l,i) = E_marg(i,l);
    }
  }

  // Permute marginal covariance back to original variable ordering
  // TODO: is this efficient?
  E_marg = perm_E * E_marg * perm_E.transpose();

  // std::cout << "Dyn Programming: " << std::endl;
  // std::cout << E_marg << std::endl;
  // std::cout << "Inverse: " << std::endl;
  // Eigen::SparseMatrix<float> R2
  //   = qr.matrixR().block(ind, ind, m-k, m-k).template triangularView<Eigen::Upper>();
  // Eigen::MatrixXf R2_dense(R2);
  // Eigen::MatrixXf E_marg_inv = (R2_dense.transpose()*R2_dense).inverse();
  // std::cout << E_marg_inv << std::endl;

  // std::cout << "Norm diff: " << std::endl;
  // std::cout << (E_marg - E_marg_inv).norm() << std::endl;

  return E_marg.bottomRightCorner(n,n);
}