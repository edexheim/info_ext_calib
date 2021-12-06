
// Partially derived from Cal3.h

/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   Cal3.h
 * @brief  Common code for all Calibration models.
 * @author Varun Agrawal
 */

#pragma once

#include <gtsam/geometry/Point2.h>

enum class CameraModelType {Pinhole, PinholeRadtan, PinholeKB};

namespace gtsam {

class GTSAM_EXPORT PinholeModel {

  protected:

    double fx_= 1.0, fy_ = 1.0; // focal length
    double s_ = 0.0; // skew
    double u0_ = 0.0, v0_ = 0.0; // principal point

    static const size_t DIM_ = 5;

  public:
    enum { dimension = DIM_ };

    typedef boost::shared_ptr<PinholeModel> shared_ptr; ///< shared pointer to calibration object

    // Constructors
    PinholeModel() = default;

    PinholeModel(double fx, double fy, double s, double u0, double v0)
      : fx_(fx), fy_(fy), s_(s), u0_(u0), v0_(v0) {}

    PinholeModel(const Vector5& v)
      : fx_(v(0)), fy_(v(1)), s_(v(2)), u0_(v(3)), v0_(v(4)) {}

    // Destructor
    virtual ~PinholeModel() {};

    // Get camera model type
    virtual CameraModelType cameraModelType() const = 0;

    /// Convert (distorted) image coordinates uv to intrinsic coordinates xy
    virtual Point2 calibrate(const Point2& p) const = 0;

    /// Convert intrinsic coordinates xy to (distorted) image coordinates uv
    virtual Point2 uncalibratePixelDeriv(const Point2& p, 
        OptionalJacobian<2, 2> Dp = boost::none) const = 0;


    /// return calibration matrix -- not really applicable
    Matrix3 K() const;

    /// return focal length - used in projection factors when point behind camera
    double fx() const {return fx_;}

    /// @}
    /// @name Testable
    /// @{

    /// print with optional string
    void print(const std::string& s = "") const;

    /// assert equality up to a tolerance
    bool equals(const PinholeModel& K, double tol = 10e-9) const;

  private:

    /** Serialization function */
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int /*version*/)
    {
      ar & BOOST_SERIALIZATION_NVP(fx_);
      ar & BOOST_SERIALIZATION_NVP(fy_);
      ar & BOOST_SERIALIZATION_NVP(s_);
      ar & BOOST_SERIALIZATION_NVP(u0_);
      ar & BOOST_SERIALIZATION_NVP(v0_);
    }

};

} // end namespace gtsam
