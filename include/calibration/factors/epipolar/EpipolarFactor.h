#pragma once

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/EssentialMatrix.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam {

class EpipolarFactor 
  : public NoiseModelFactor2<Pose3, Pose3> {

protected:
  Point3 r1_, r2_;
  bool use_sampson_;

public:

  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<This> shared_ptr;
  
  /** default constructor - only use for serialization */
  EpipolarFactor() {}

  /**
   * Constructor
   * Base frames of poses don't matter as long as consistent
   * @param T_c1_key is the key of camera 1
   * @param T_c2_key is the key of camera 2
   * @param r1 is the camera 1 feature observation bearing vector
   * @param r2 is the camera 1 feature observation bearing vector
   * @param model is the standard deviation
   */
  EpipolarFactor( const Key& T_c1_key, const Key& T_c2_key,
                  const Point3& r1, const Point3& r2,
                  bool use_sampson,
                  const SharedNoiseModel& model)
                  : NoiseModelFactor2<Pose3, Pose3>(model, T_c1_key, T_c2_key),
                  r1_(r1), r2_(r2), use_sampson_(use_sampson)
                  {}

  // Virtual destructor
  virtual ~EpipolarFactor() {};

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new EpipolarFactor(*this)));
  }

  // TODO: Implement Sampson error as well
  Vector algebraicError(Pose3 T_c1, Pose3 T_c2) const {
    Pose3 T_c1_c2 = T_c1.between(T_c2);
    // EssentialMatrix E = EssentialMatrix::FromPose3(T_c1_c2);
    Matrix3 E = skewSymmetric(T_c1_c2.translation()) * T_c1_c2.rotation().matrix();
    double error = r1_.dot(E*r2_);
    return (Vector(1) << error).finished();  
  }

  static double SampsonError(Matrix3 E, Point3 r1, Point3 r2) {
    double numer = r1.dot(E*r2);
    double l1_0 = E.col(0).dot(r1);
    double l1_1 = E.col(1).dot(r1);
    double l2_0 = E.row(0).dot(r2);
    double l2_1 = E.row(1).dot(r2);
    double tmp = l1_0*l1_0 + l1_1*l1_1 + l2_0*l2_0 + l2_1*l2_1;
    double denom = std::sqrt(tmp);
    // Watch undefined behavior at epipoles
    return (denom > 1e-3) ? numer/denom : 0;  
  }

  static double SampsonErrorSq(Matrix3 E, Point3 r1, Point3 r2) {
    double algebraic = r1.dot(E*r2);
    double numer = algebraic*algebraic;
    double l1_0 = E.col(0).dot(r1);
    double l1_1 = E.col(1).dot(r1);
    double l2_0 = E.row(0).dot(r2);
    double l2_1 = E.row(1).dot(r2);
    double denom = l1_0*l1_0 + l1_1*l1_1 + l2_0*l2_0 + l2_1*l2_1;
    return numer/denom;  
  }

  Vector sampsonError(Pose3 T_c1, Pose3 T_c2) const {
    Pose3 T_c1_c2 = T_c1.between(T_c2);
    Matrix3 E = skewSymmetric(T_c1_c2.translation()) * T_c1_c2.rotation().matrix();
    double e = SampsonError(E, r1_, r2_);
    return (Vector(1) << e).finished();  
  }

  Vector evaluateError(const Pose3& T_c1, const Pose3& T_c2,
      boost::optional<Matrix&> H1 = boost::none,
      boost::optional<Matrix&> H2 = boost::none) const override {

    Vector e;

    if (use_sampson_) {
      auto err = [&]( const gtsam::Pose3& T1, const gtsam::Pose3& T2) {
        return sampsonError(T1, T2);
      };
      e = err(T_c1, T_c2);
      if (H1) {
        (*H1) = numericalDerivative21<Vector, Pose3, Pose3>(err, T_c1, T_c2);
      }
      if (H2) {
        (*H2) = numericalDerivative22<Vector, Pose3, Pose3>(err, T_c1, T_c2);
      }
    }
    else {
      auto err = [&]( const gtsam::Pose3& T1, const gtsam::Pose3& T2) {
        return algebraicError(T1, T2);
      };
      e = err(T_c1, T_c2);
      if (H1) {
        (*H1) = numericalDerivative21<Vector, Pose3, Pose3>(err, T_c1, T_c2);
      }
      if (H2) {
        (*H2) = numericalDerivative22<Vector, Pose3, Pose3>(err, T_c1, T_c2);
      }
      // std::cout << e << std::endl;
      // std::cout << (*H1) << std::endl;
      // std::cout << (*H2) << std::endl;

      // Matrix36 H_R1, H_t1;
      // Rot3 R1 = T_c1.rotation(H_R1);
      // Point3 t1 = T_c1.translation(H_t1);
      // Point3 u1 = R1*r1_;

      // Matrix36 H_R2, H_t2;
      // Rot3 R2 = T_c2.rotation(H_R2);
      // Point3 t2 = T_c2.translation(H_t2);
      // Point3 u2 = R2*r2_;

      // // Algebraic error
      // Matrix3 tx = skewSymmetric(t2-t1);
      // Point3 tmp1 = R1.transpose() * tx * u2;
      // double algebraic = r1_.dot(tmp1);

      // // Jacobians
      // if (H1 || H2) {
      //   Eigen::RowVector3d H1_R, H2_R, H1_t, H2_t;
      //   H1_R = r1_.transpose() * skewSymmetric(tmp1);
      //   H2_R = -u1.transpose() * tx * R2.matrix() * skewSymmetric(r2_);
      //   H1_t = (u1.cross(u2)).transpose();
      //   H2_t = -H1_t;
      //   *H1 = H1_R * H_R1 + H1_t * H_t1;
      //   *H2 = H2_R * H_R2 + H2_t * H_t2;
      // }
      // e = (Vector(1) << algebraic).finished();
      // std::cout << e << std::endl;
      // std::cout << (*H1) << std::endl;
      // std::cout << (*H2) << std::endl;
    }

    // if (use_sampson_) {
    //   // Sampson error
    //   Matrix3 E = R1.transpose() * tx * R2.matrix();

    //   double l1_0 = E.col(0).dot(r1_);
    //   double l1_1 = E.col(1).dot(r1_);
    //   double l2_0 = E.row(0).dot(r2_);
    //   double l2_1 = E.row(1).dot(r2_);
    //   double b = l1_0*l1_0 + l1_1+l1_1 + l2_0*l2_0 + l2_1*l2_1;
    //   double b_inv = 1.0/b;
    //   double b_sqrt_inv = std::sqrt(b_inv);
    //   e = (Vector(1) << algebraic*b_sqrt_inv).finished();

    //   // Jacobians
    //   if (H1 || H2) {
    //     double b3_sqrt_inv = b_inv * b_sqrt_inv;

    //     Eigen::RowVector3d db_dR2 = 2*(E.row(0)*r2)
    //     H1_R = b_sqrt_inv * H1_R - algebraic/2 * b3_sqrt_inv * db_dR;
    //   }
    // }

    return e;
  }

private:
  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(NoiseModelFactor2);
    ar & BOOST_SERIALIZATION_NVP(r1_);
    ar & BOOST_SERIALIZATION_NVP(r2_);
    ar & BOOST_SERIALIZATION_NVP(use_sampson_);
  }
};

} // end namespace gtsam
