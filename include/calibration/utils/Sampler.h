/**
 * @file   Sampler.h
 * @brief  Class to sample from Gaussian distribution of Lie Groups
 * @author Eric Dexheimer
 */

// Largely derived from Sampler.cpp
// Original file license and authors:

/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file Sampler.h
 * @brief sampling from a NoiseModel
 * @author Frank Dellaert
 * @author Alex Cunningham
 */

#pragma once

#include <gtsam/base/GenericValue.h>
#include <gtsam/linear/NoiseModel.h>

#include <random>

#include <Eigen/Cholesky>

namespace gtsam {

template<class T>
class Sampler {

  protected:

    static const int Dim = traits<T>::dimension;
    typedef Eigen::Matrix<double, Dim, 1> Vec;
    typedef Eigen::Matrix<double, Dim, Dim> SqMat;

    SqMat L_;

    /** generator */
    mutable std::mt19937_64 generator_;

  public:
    typedef boost::shared_ptr<Sampler> shared_ptr;

    /// @name constructors
    /// @{

    /**
     * NOTE: do not use zero as a seed, it will break the generator
     */

    explicit Sampler(const SqMat& cov, uint64_t seed = 42u) 
        : generator_(seed) {

      // TODO: Assertions
      Eigen::LLT<SqMat> llt(cov);
      L_ = llt.matrixL();
    }

    explicit Sampler(const noiseModel::Gaussian::shared_ptr& model, 
        uint64_t seed = 42u) 
        : Sampler(SqMat(model->covariance()), seed) {}

    explicit Sampler(const Vec& sigmas, uint64_t seed = 42u) 
        : Sampler(SqMat((sigmas.cwiseProduct(sigmas)).asDiagonal()) ) {}

    /// @}
    /// @name access functions
    /// @{

    size_t dim() const {
      return Dim;
    }

    /// @}
    /// @name basic functionality
    /// @{

    /// sample from distribution
    T sample(const T& mean) const {
      Vec delta;
      for (size_t i = 0; i < Dim; i++) {
        typedef std::normal_distribution<double> Normal;
        Normal dist(0.0, 1.0);
        delta(i) = dist(generator_);
      }
      GenericValue<T> result = GenericValue<T>(mean).retract(L_*delta);
      return result.value();
    }

    /// @}

};

}  // namespace gtsam