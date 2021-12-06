#ifndef GENEPIFACTOR_H
#define GENEPIFACTOR_H

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/Matrix.h>

namespace gtsam {

class GenEpiFactor 
  : public NoiseModelFactor6<Rot3, Point3, Rot3, Point3, Rot3, Point3> {

public:
  
  /** default constructor - only use for serialization */
  GenEpiFactor() {}

  /**
   * Constructor
   * @param Rv_i_key is the index of absolute vehicle rotation at time i
   * @param tv_i_key is the index of absolute vehicle translation at time i
   * @param Rv_j_key is the index of absolute vehicle rotation at time j
   * @param tv_j_key is the index of absolute vehicle translation at time j
   * @param Rvc_key is the index of extrinsic rotation from cam-to-vehicle
   * @param tvc_key is the index of extrinsic translation from cam-to-vehicle
   * @param ri is the camera bearing vector of feature observation at time i
   * @param rj is the camera bearing vector of feature observation at time j
   * @param model is the standard deviation
   */
  GenEpiFactor( const Key& Rv_i_key, const Key& tv_i_key,
                const Key& Rv_j_key, const Key& tv_j_key,
                const Key& Rvc_key, const Key& tvc_key, 
                const Point3& ri, const Point3& rj,
                const SharedNoiseModel& model)
                : NoiseModelFactor6<Rot3, Point3, Rot3, Point3, Rot3, Point3>
                    (model, Rv_i_key, tv_i_key, 
                            Rv_j_key, tv_j_key,
                            Rvc_key, tvc_key),
                ri_(ri), rj_(rj)
                {}

  // Virtual destructor
  virtual ~GenEpiFactor() {};

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new GenEpiFactor(*this)));
  }

  Vector genEpiError( Rot3 Rv_i, Point3 tv_i,
                      Rot3 Rv_j, Point3 tv_j,
                      Rot3 Rvc,  Point3 tvc) const {

    Matrix3 ti_x = skewSymmetric(tv_i);
    Matrix3 tj_x = skewSymmetric(tv_j);
    Matrix3 tij_x = tj_x - ti_x;
    Matrix3 tvc_x = skewSymmetric(tvc);
    Point3 ui = Rvc*ri_;
    Point3 uj = Rvc*rj_;
    Matrix3 Rv_i_inv = Rv_i.transpose();
    Matrix3 Rv_ij = Rv_i_inv * Rv_j.matrix();

    Matrix3 E1 = Rv_i_inv * tij_x * Rv_j.matrix();
    Matrix3 E2 = -tvc_x * Rv_ij.matrix();
    Matrix3 E3 = Rv_ij * tvc_x;
    Matrix3 Ev = E1 + E2 + E3;
    double f = ui.dot(Ev * uj);
    return (Vector(1) << f).finished();
  }

  // Helper to evaluate error h(x)-z and optionally derivatives
  static Vector getError( const Rot3& Rv_i, const Point3& tv_i,
                          const Rot3& Rv_j, const Point3& tv_j,
                          const Rot3& Rvc, const Point3& tvc,
                          const Point3& r_i, const Point3& r_j,
                          boost::optional<Matrix&> H1=boost::none,
                          boost::optional<Matrix&> H2=boost::none,
                          boost::optional<Matrix&> H3=boost::none,
                          boost::optional<Matrix&> H4=boost::none,
                          boost::optional<Matrix&> H5=boost::none,
                          boost::optional<Matrix&> H6=boost::none) {
    // auto err = [&]( const gtsam::Rot3& Rv1, const gtsam::Point3& tv1,
    //                 const gtsam::Rot3& Rv2, const gtsam::Point3& tv2,
    //                 const gtsam::Rot3& Rc, const gtsam::Point3& tc) {
    //   return genEpiError(Rv1, tv1, Rv2, tv2, Rc, tc); 
    // };

    Matrix3 ti_x = skewSymmetric(tv_i);
    Matrix3 tj_x = skewSymmetric(tv_j);
    Matrix3 tij_x = tj_x - ti_x;
    Matrix3 tvc_x = skewSymmetric(tvc);
    Point3 ui = Rvc*r_i;
    Point3 uj = Rvc*r_j;
    Matrix3 Rv_i_inv = Rv_i.transpose();
    Matrix3 Rv_ij = Rv_i_inv * Rv_j.matrix();

    Matrix3 E1 = Rv_i_inv * tij_x * Rv_j.matrix();
    Matrix3 E2 = -tvc_x * Rv_ij.matrix();
    Matrix3 E3 = Rv_ij * tvc_x;
    Matrix3 Ev = E1 + E2 + E3;
    double f = ui.dot(Ev * uj);

    if (H1 || H3) {
      Matrix3 uj_x = skewSymmetric(uj);
      Point3 J_f_Rj_tmp = -ui.transpose()*(E2*uj_x + Rv_ij*skewSymmetric(tvc_x*uj));
      if (H1) (*H1) = Eigen::RowVector3d( ui.transpose()*(skewSymmetric(E1*uj)) - J_f_Rj_tmp.transpose()*Rv_ij.transpose() );
      if (H3) (*H3) = Eigen::RowVector3d( (-ui.transpose()*(E1*uj_x)).transpose() + J_f_Rj_tmp );
    }
    if (H2 || H4) {
      Point3 J_tv_i = (Rv_i*ui).cross(Rv_j*uj);
      if (H2) (*H2) = Eigen::RowVector3d(J_tv_i);
      if (H4) (*H4) = -Eigen::RowVector3d(J_tv_i);
    }
    if (H5) {
      // TODO: any way to do analytically?
      Point3 J_Rvc;
      float h = 1e-5;
      float sin_h = std::sin(h);
      float cos_h = 1-std::cos(h);
      for (int i=0; i<3; i++) {
        Point3 w = Point3::Zero();
        w(i) = 1.0;
        // TODO: can precalculate, simplify since known beforehand
        Matrix3 w_hat = skewSymmetric(w);
        Matrix3 w_hat_sq = -Matrix3::Identity();
        w_hat_sq(i,i) = 0;
        // Rodrigues
        Matrix3 R_inc = Matrix3::Identity() 
                                + w_hat*sin_h + w_hat_sq*cos_h;
        Matrix3 Rvc_new = Rvc.matrix()*R_inc;
        J_Rvc(i) = ( (Rvc_new*r_i).dot(Ev*Rvc_new*r_j) - f )/h;
      }
      (*H5) = Eigen::RowVector3d(J_Rvc);
    }
    if (H6) {
      (*H6) = Eigen::RowVector3d( ui.cross(Rv_ij*uj) - (Rv_ij.transpose()*ui).cross(uj) );
    }

    // if (H1) {
    //   (*H1) 
    //   = numericalDerivative61<Vector, Rot3, Point3, Rot3, Point3, Rot3, Point3>
    //       (err, Rv_i, tv_i, Rv_j, tv_j, Rvc, tvc);
    // }
    // if (H2) {
    //   (*H2) 
    //   = numericalDerivative62<Vector, Rot3, Point3, Rot3, Point3, Rot3, Point3>
    //       (err, Rv_i, tv_i, Rv_j, tv_j, Rvc, tvc);
    // }
    // if (H3) {
    //   (*H3) 
    //   = numericalDerivative63<Vector, Rot3, Point3, Rot3, Point3, Rot3, Point3>
    //       (err, Rv_i, tv_i, Rv_j, tv_j, Rvc, tvc);
    // }
    // if (H4) {
    //   (*H4) 
    //   = numericalDerivative64<Vector, Rot3, Point3, Rot3, Point3, Rot3, Point3>
    //       (err, Rv_i, tv_i, Rv_j, tv_j, Rvc, tvc);
    // }
    // if (H5) {
    //   (*H5) 
    //   = numericalDerivative65<Vector, Rot3, Point3, Rot3, Point3, Rot3, Point3>
    //       (err, Rv_i, tv_i, Rv_j, tv_j, Rvc, tvc);
    // }
    // if (H6) {
    //   (*H6) 
    //   = numericalDerivative66<Vector, Rot3, Point3, Rot3, Point3, Rot3, Point3>
    //       (err, Rv_i, tv_i, Rv_j, tv_j, Rvc, tvc);
    // }

    // return err(Rv_i, tv_i, Rv_j, tv_j, Rvc, tvc);
    return (Vector(1) << f).finished();
  }

  Vector evaluateError( const Rot3& Rv_i, const Point3& tv_i,
                        const Rot3& Rv_j, const Point3& tv_j,
                        const Rot3& Rvc, const Point3& tvc,
                        boost::optional<Matrix&> H1=boost::none,
                        boost::optional<Matrix&> H2=boost::none,
                        boost::optional<Matrix&> H3=boost::none,
                        boost::optional<Matrix&> H4=boost::none,
                        boost::optional<Matrix&> H5=boost::none,
                        boost::optional<Matrix&> H6=boost::none) const override {

    return this->getError(Rv_i, tv_i, Rv_j, tv_j, Rvc, tvc, ri_, rj_,
                          H1, H2, H3, H4, H5, H6);
  }

protected:
  Point3 ri_, rj_;

private:
  /** Serialization function */
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(NoiseModelFactor6);
    ar & BOOST_SERIALIZATION_NVP(ri_);
    ar & BOOST_SERIALIZATION_NVP(rj_);
  }
};

} // end namespace gtsam

#endif