
// Partially derived from Cal3_S2.h

/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   Cal3_S2.h
 * @brief  The most common 5DOF 3D->2D calibration
 * @author Frank Dellaert
 */

#pragma once

#include "calibration/camera_models/PinholeModel.h"
#include <gtsam/base/OptionalJacobian.h>

namespace gtsam {

class GTSAM_EXPORT Pinhole : public PinholeModel {

  protected:

    static const size_t DIM_ = 5; //PinholeModel::DIM_ + 0;

  public:
    enum { dimension = DIM_ };

    typedef boost::shared_ptr<Pinhole> shared_ptr; ///< shared pointer to calibration object

    // Constructors
    Pinhole() = default;

    Pinhole(double fx, double fy, double s, double u0, double v0)
      : PinholeModel(fx, fy, s, u0, v0) {}

    Pinhole(const Vector5& v)
      : PinholeModel(v) {}

    // Destructor
    virtual ~Pinhole() {};

    // Get camera model type
    virtual CameraModelType cameraModelType() const override
       {return CameraModelType::Pinhole;}

    /// Get vector of intrinsics
    Vector5 vector() const;

    /// Convert (distorted) image coordinates uv to intrinsic coordinates xy
    Point2 calibrate(const Point2& p) const override;

    /// Convert intrinsic coordinates xy to (distorted) image coordinates uv
    virtual Point2 uncalibratePixelDeriv(const Point2& p,
        OptionalJacobian<2, 2> Dp = boost::none) const override {
      return this->uncalibrate(p, boost::none, Dp);
    }

    /**
    * Convert intrinsic coordinates xy to image coordinates uv, fixed derivaitves
    * @param p point in intrinsic coordinates
    * @param Dcal optional 2*5 Jacobian wrpt Cal3 parameters
    * @param Dp optional 2*2 Jacobian wrpt intrinsic coordinates
    * @return point in image coordinates
    */
    Point2 uncalibrate(const Point2& p, OptionalJacobian<2, DIM_> Dcal = boost::none,
                      OptionalJacobian<2, 2> Dp = boost::none) const;

    /// @}
    /// @name Testable
    /// @{

    /// print with optional string
    void print(const std::string& s = "") const;

    /// assert equality up to a tolerance
    bool equals(const Pinhole& K, double tol = 10e-9) const;

    /// Return dimensions of calibration manifold object
    virtual size_t dim() const { return DIM_; }

    /// @}
    /// @name Manifold
    /// @{

    Pinhole retract(const Vector& d) const;

    Vector localCoordinates(const Pinhole& T2) const;

  private:

    /** Serialization function */
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int /*version*/)
    {
      ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(PinholeModel);
    }

};

template <>
struct traits<Pinhole> : public internal::Manifold<Pinhole> {};

template <>
struct traits<const Pinhole> : public internal::Manifold<Pinhole> {};

} // end namespace gtsam
