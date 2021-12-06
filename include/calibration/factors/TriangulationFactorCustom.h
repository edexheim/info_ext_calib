/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * triangulationFactor.h
 * @date March 2, 2014
 * @author Frank Dellaert
 */

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <boost/make_shared.hpp>
#include <boost/lexical_cast.hpp>

#include "calibration/camera_models/PinholeModel.h"

namespace gtsam {

/**
 * Non-linear factor for a constraint derived from a 2D measurement.
 * The calibration and pose are assumed known.
 * @addtogroup SLAM
 */
class TriangulationFactorCustom: public NoiseModelFactor1<Point3> {

protected:

  /// shorthand for base class type
  typedef NoiseModelFactor1<Point3> Base;
  typedef TriangulationFactorCustom This;

  // Keep a copy of measurement and calibration for I/O
  Point2 measured_;                    ///< 2D measurement
  Pose3 pose_;                         ///< 6 DoF camera pose
  boost::shared_ptr<PinholeModel> K_;  ///< shared pointer to calibration object

  // verbosity handling for Cheirality Exceptions
  const bool throwCheirality_; ///< If true, rethrows Cheirality exceptions (default: false)
  const bool verboseCheirality_; ///< If true, prints text for Cheirality exceptions (default: false)

public:

  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<This> shared_ptr;

  /// Default constructor
  TriangulationFactorCustom() :
      throwCheirality_(false), verboseCheirality_(false) {
  }

  /**
   * Constructor with exception-handling flags
   * @param camera is the camera in which unknown landmark is seen
   * @param measured is the 2 dimensional location of point in image (the measurement)
   * @param model is the standard deviation
   * @param pointKey is the index of the landmark
   * @param throwCheirality determines whether Cheirality exceptions are rethrown
   * @param verboseCheirality determines whether exceptions are printed for Cheirality
   */
  TriangulationFactorCustom(
      const Point2& measured,
      const Pose3& pose,
      const boost::shared_ptr<PinholeModel>& K,
      const SharedNoiseModel& model, Key pointKey, 
      bool throwCheirality = false,
      bool verboseCheirality = false) :
      Base(model, pointKey), 
        measured_(measured), pose_(pose), K_(K),
        throwCheirality_(throwCheirality), 
        verboseCheirality_(verboseCheirality) {
    if (model && model->dim() != traits<Point2>::dimension)
      throw std::invalid_argument(
          "TriangulationFactorCustom must be created with "
              + boost::lexical_cast<std::string>((int) traits<Point2>::dimension)
              + "-dimensional noise model.");
  }

  /** Virtual destructor */
  ~TriangulationFactorCustom() override {
  }

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /**
   * print
   * @param s optional string naming the factor
   * @param keyFormatter optional formatter useful for printing Symbols
   */
  void print(const std::string& s = "", const KeyFormatter& keyFormatter =
      DefaultKeyFormatter) const override {
    std::cout << s << "TriangulationFactorCustom,";
    pose_.print("pose");
    K_->print("camera");
    traits<Point2>::Print(measured_, "z");
    Base::print("", keyFormatter);
  }

  /// equals
  bool equals(const NonlinearFactor& p, double tol = 1e-9) const override {
    const This *e = dynamic_cast<const This*>(&p);
    return e && Base::equals(p, tol) 
        && traits<Point2>::Equals(this->measured_, e->measured_, tol)
        && this->pose_.equals(e->pose_, tol)
        && this->K_->equals(*e->K_, tol);
  }

  /// Evaluate error h(x)-z and optionally derivatives
  Vector evaluateError(const Point3& point, boost::optional<Matrix&> H2 =
      boost::none) const override {
    
    try {
      gtsam::PinholeBase cam(pose_);
      Matrix23 D_pn_pw;
      const Point2 pn = cam.project2(point, boost::none, H2 ? &D_pn_pw  : 0);
      Matrix2 D_pi_pn;
      const Point2 pi = K_->uncalibratePixelDeriv(pn, H2 ? &D_pi_pn : 0);
      if (H2)
        *H2 = D_pi_pn * D_pn_pw;

      return pi - measured_;
    } catch (CheiralityException& e) {
      if (H2)
        *H2 = Matrix::Zero(traits<Point2>::dimension, 3);
      if (verboseCheirality_)
        std::cout << e.what() << ": Landmark "
            << DefaultKeyFormatter(this->key()) << " moved behind camera"
            << std::endl;
      if (throwCheirality_)
        throw e;
      return Vector::Ones(2) * 2.0 * K_->fx();
    }
  }

  /** return the measurement */
  const Point2& measured() const {
    return measured_;
  }

  /** return the pose */
  const Pose3& pose() const {
    return pose_;
  }

  /** return verbosity */
  inline bool verboseCheirality() const {
    return verboseCheirality_;
  }

  /** return flag for throwing cheirality exceptions */
  inline bool throwCheirality() const {
    return throwCheirality_;
  }

private:

  /// Serialization function
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar & BOOST_SERIALIZATION_NVP(measured_);
    ar & BOOST_SERIALIZATION_NVP(pose_);
    ar & BOOST_SERIALIZATION_NVP(K_);
    ar & BOOST_SERIALIZATION_NVP(throwCheirality_);
    ar & BOOST_SERIALIZATION_NVP(verboseCheirality_);
  }
};
} // \ namespace gtsam