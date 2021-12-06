// Derived from ProjectionFactorPPP.h, but NOT templated on calibration
// Assumes camera model is derived from PinholeModel
// Original license below

/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file ProjectionFactorPPP.h
 * @brief Derived from ProjectionFactor, but estimates body-camera transform
 * in addition to body pose and 3D landmark
 * @author Chris Beall
 */

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include "calibration/camera_models/PinholeModel.h"
#include <boost/optional.hpp>
#include <gtsam/geometry/CalibratedCamera.h>

namespace gtsam {

  /**
   * Non-linear factor for a constraint derived from a 2D measurement. The calibration is known here.
   * i.e. the main building block for visual SLAM.
   * @addtogroup SLAM
   */
  template<class POSE, class LANDMARK>
  class ExtrinsicsProjectionFactor: public NoiseModelFactor3<POSE, POSE, LANDMARK> {
  protected:

    // Keep a copy of measurement and calibration for I/O
    Point2 measured_;                    ///< 2D measurement
    boost::shared_ptr<PinholeModel> K_;  ///< shared pointer to calibration object

    // verbosity handling for Cheirality Exceptions
    bool throwCheirality_; ///< If true, rethrows Cheirality exceptions (default: false)
    bool verboseCheirality_; ///< If true, prints text for Cheirality exceptions (default: false)

  public:

    /// shorthand for base class type
    typedef NoiseModelFactor3<POSE, POSE, LANDMARK> Base;

    /// shorthand for this class
    typedef ExtrinsicsProjectionFactor<POSE, LANDMARK> This;

    /// shorthand for a smart pointer to a factor
    typedef boost::shared_ptr<This> shared_ptr;

    /// Default constructor
  ExtrinsicsProjectionFactor() :
      measured_(0.0, 0.0), throwCheirality_(false), verboseCheirality_(false) {
  }

    /**
     * Constructor
     * TODO: Mark argument order standard (keys, measurement, parameters)
     * @param measured is the 2 dimensional location of point in image (the measurement)
     * @param model is the standard deviation
     * @param poseKey is the index of the camera
     * @param transformKey is the index of the body-camera transform
     * @param pointKey is the index of the landmark
     * @param K shared pointer to the constant calibration
     */
    ExtrinsicsProjectionFactor(const Point2& measured, const SharedNoiseModel& model,
        Key poseKey, Key transformKey,  Key pointKey,
        const boost::shared_ptr<PinholeModel>& K) :
          Base(model, poseKey, transformKey, pointKey), measured_(measured), K_(K),
          throwCheirality_(false), verboseCheirality_(false) {}

    /**
     * Constructor with exception-handling flags
     * TODO: Mark argument order standard (keys, measurement, parameters)
     * @param measured is the 2 dimensional location of point in image (the measurement)
     * @param model is the standard deviation
     * @param poseKey is the index of the camera
     * @param pointKey is the index of the landmark
     * @param K shared pointer to the constant calibration
     * @param throwCheirality determines whether Cheirality exceptions are rethrown
     * @param verboseCheirality determines whether exceptions are printed for Cheirality
     */
    ExtrinsicsProjectionFactor(const Point2& measured, const SharedNoiseModel& model,
        Key poseKey, Key transformKey, Key pointKey,
        const boost::shared_ptr<PinholeModel>& K,
        bool throwCheirality, bool verboseCheirality) :
          Base(model, poseKey, transformKey, pointKey), measured_(measured), K_(K),
          throwCheirality_(throwCheirality), verboseCheirality_(verboseCheirality) {}

    /** Virtual destructor */
    ~ExtrinsicsProjectionFactor() override {}

    /// @return a deep copy of this factor
    NonlinearFactor::shared_ptr clone() const override {
      return boost::static_pointer_cast<NonlinearFactor>(
          NonlinearFactor::shared_ptr(new This(*this))); }

    /**
     * print
     * @param s optional string naming the factor
     * @param keyFormatter optional formatter useful for printing Symbols
     */
    void print(const std::string& s = "", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
      std::cout << s << "ExtrinsicsProjectionFactor, z = ";
      traits<Point2>::Print(measured_);
      Base::print("", keyFormatter);
    }

    /// equals
    bool equals(const NonlinearFactor& p, double tol = 1e-9) const override {
      const This *e = dynamic_cast<const This*>(&p);
      return e
          && Base::equals(p, tol)
          && traits<Point2>::Equals(this->measured_, e->measured_, tol)
          && this->K_->equals(*e->K_, tol);
    }

    /// Evaluate error h(x)-z and optionally derivatives
    Vector evaluateError(const Pose3& pose, const Pose3& transform, const Point3& point_w,
        boost::optional<Matrix&> H1 = boost::none,
        boost::optional<Matrix&> H2 = boost::none,
        boost::optional<Matrix&> H3 = boost::none) const override {
      
      bool pose_deriv = (H1 || H2);
      bool calc_deriv = (pose_deriv || H3);
      
      try {
          if (calc_deriv) {
            Matrix6 D_Twc_Twb, D_Twc_Tbc;
            Pose3 T_wc = pose.compose(transform, 
                H1 ? &D_Twc_Twb : 0, 
                H2 ? &D_Twc_Tbc : 0);
            gtsam::PinholeBase cam(T_wc);
            Matrix26 D_pn_Twc;
            Matrix23 D_pn_pw;
            const Point2 pn = cam.project2(point_w, 
                pose_deriv ? &D_pn_Twc : 0,
                calc_deriv ? &D_pn_pw  : 0);
            Matrix2 D_pi_pn;
            const Point2 pi = K_->uncalibratePixelDeriv(pn, calc_deriv ? &D_pi_pn : 0);

            if (pose_deriv) {
              Matrix26 D_pi_Twc = D_pi_pn * D_pn_Twc;
              if (H1) 
                *H1 = D_pi_Twc * D_Twc_Twb;
              if (H2) 
                *H2 = D_pi_Twc * D_Twc_Tbc;
            }
            if (H3) 
              *H3 = D_pi_pn * D_pn_pw;

            Point2 reprojectionError(pi - measured_);
            return reprojectionError;
          } else {
            Pose3 T_wc = pose.compose(transform);
            gtsam::PinholeBase cam(T_wc);
            const Point2 pn = cam.project2(point_w);
            const Point2 pi = K_->uncalibratePixelDeriv(pn);
            Point2 reprojectionError(pi - measured_);
            return reprojectionError;
          }
      } catch( CheiralityException& e) {
        if (H1) *H1 = Matrix::Zero(2,6);
        if (H2) *H2 = Matrix::Zero(2,6);
        if (H3) *H3 = Matrix::Zero(2,3);
        if (verboseCheirality_)
          std::cout << e.what() << ": Landmark "<< DefaultKeyFormatter(this->key2()) <<
              " moved behind camera " << DefaultKeyFormatter(this->key1()) << std::endl;
        if (throwCheirality_)
          throw e;
      }
      return Vector::Ones(2) * 2.0 * K_->fx();
    }

    /** return the measurement */
    const Point2& measured() const {
      return measured_;
    }

    /** return the calibration object */
    inline const boost::shared_ptr<PinholeModel> calibration() const {
      return K_;
    }

    /** return verbosity */
    inline bool verboseCheirality() const { return verboseCheirality_; }

    /** return flag for throwing cheirality exceptions */
    inline bool throwCheirality() const { return throwCheirality_; }

  private:

    /// Serialization function
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
      ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
      ar & BOOST_SERIALIZATION_NVP(measured_);
      ar & BOOST_SERIALIZATION_NVP(K_);
      ar & BOOST_SERIALIZATION_NVP(throwCheirality_);
      ar & BOOST_SERIALIZATION_NVP(verboseCheirality_);
    }
  };

  /// traits
  template<class POSE, class LANDMARK>
  struct traits<ExtrinsicsProjectionFactor<POSE, LANDMARK> > :
      public Testable<ExtrinsicsProjectionFactor<POSE, LANDMARK> > {
  };

} // \ namespace gtsam