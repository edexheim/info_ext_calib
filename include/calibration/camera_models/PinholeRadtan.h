// Largely derived from Cal3DS2_Base, but now inheriting from PinholeModel
// Also exposes max iterations and tolernace as members for calibrate calls

/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file Cal3DS2.h
 * @brief Calibration of a camera with radial distortion
 * @date Feb 28, 2010
 * @author ydjian
 * @author Varun Agrawal
 */

#pragma once

#include "calibration/camera_models/PinholeModel.h"
#include "calibration/frontend/FrontendParams.h"

namespace gtsam {

class GTSAM_EXPORT PinholeRadtan : public PinholeModel {

  protected:

    // TODO: Remove skew?
    double k1_ = 0.0, k2_ = 0.0; // radial 2nd-order and 4th-order
    double p1_ = 0.0, p2_ = 0.0; // tangential distortion

    FrontendParams::UndistortParams calibrate_params_;

    static const size_t DIM_ = 9; //PinholeModel::DIM_ + 4;

  public:

    enum { dimension = DIM_ };

    typedef boost::shared_ptr<PinholeRadtan> shared_ptr; ///< shared pointer to calibration object

    // Constructors
    PinholeRadtan() = default;

    PinholeRadtan(const Vector& v, const FrontendParams::UndistortParams& params);

    // Destructor
    virtual ~PinholeRadtan() {}

    // Get camera model type
    virtual CameraModelType cameraModelType() const override
      {return CameraModelType::PinholeRadtan;}

    // Get distortion coeffs
    Vector4 k() const;

    /// Get vector of intrinsics and distortion coeffs
    Vector9 vector() const;


    /// Convert (distorted) image coordinates uv to intrinsic coordinates xy
    Point2 calibrate(const Point2& p) const override;

    /// Convert intrinsic coordinates xy to (distorted) image coordinates uv
    virtual Point2 uncalibratePixelDeriv(const Point2& p,
        OptionalJacobian<2, 2> Dp = boost::none) const override {
      return this->uncalibrate(p, boost::none, Dp);
    }

    /**
     * convert intrinsic coordinates xy to (distorted) image coordinates uv
     * @param p point in intrinsic coordinates
     * @param Dcal optional Jacobian wrpt PinholeRadtan parameters
     * @param Dp optional Jacobian wrpt intrinsic coordinates
     * @return point in (distorted) image coordinates
     */
    Point2 uncalibrate(const Point2& p,
      OptionalJacobian<2, DIM_> Dcal = boost::none,
      OptionalJacobian<2, 2> Dp = boost::none) const;

    /// @}
    /// @name Testable
    /// @{

    /// print with optional string
    void print(const std::string& s = "") const;

    /// assert equality up to a tolerance
    bool equals(const PinholeRadtan& K, double tol = 10e-9) const;

    /// Return dimensions of calibration manifold object
    virtual size_t dim() const { return DIM_; }

    /// @}
    /// @name Manifold
    /// @{

    PinholeRadtan retract(const Vector& d) const;

    Vector localCoordinates(const PinholeRadtan& T2) const;

  private:

    /// Derivative of uncalibrate wrpt intrinsic coordinates
    Matrix2 D2d_intrinsic(const Point2& p) const;

    /// Derivative of uncalibrate wrpt the calibration parameters
    Matrix29 D2d_calibration(const Point2& p) const;

    /** Serialization function */
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int /*version*/)
    {
      ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(PinholeModel);
      ar & BOOST_SERIALIZATION_NVP(k1_);
      ar & BOOST_SERIALIZATION_NVP(k2_);
      ar & BOOST_SERIALIZATION_NVP(p1_);
      ar & BOOST_SERIALIZATION_NVP(p2_);
      ar & BOOST_SERIALIZATION_NVP(calibrate_params_);
    }

};

template<>
struct traits<PinholeRadtan> : public internal::Manifold<PinholeRadtan> {};

template<>
struct traits<const PinholeRadtan> : public internal::Manifold<PinholeRadtan> {};

} // end namespace gtsam
