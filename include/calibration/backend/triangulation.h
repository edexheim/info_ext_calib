// Modified from GTSAM <gtsam/geometry/triangulation.h>
// Original license

/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file triangulation.h
 * @brief Functions for triangulation
 * @date July 31, 2013
 * @author Chris Beall
 */

#pragma once

#include <gtsam/geometry/triangulation.h>
#include "calibration/utils/basic_types.h"

namespace gtsam {

Vector4 triangulateHomogeneousDLTCustom(
    const std::vector<Matrix34, Eigen::aligned_allocator<Matrix34>>& projection_matrices,
    const Point2Vector& measurements, double rank_tol); 

Point3 triangulateDLTCustom(const std::vector<Matrix34, Eigen::aligned_allocator<Matrix34>>& projection_matrices,
    const Point2Vector& measurements, double rank_tol); 

std::pair<NonlinearFactorGraph, Values> triangulationGraphCustom(
    const std::vector<PinholeModel::shared_ptr>& intrinsics,
    const std::vector<Pose3>& poses,
    const Point2Vector& measurements,
    Key landmarkKey,
    const Point3& initialEstimate,
    SharedNoiseModel noise_model);

/**
 * Optimize for triangulation
 * @param graph nonlinear factors for projection
 * @param values initial values
 * @param landmarkKey to refer to landmark
 * @param rank_tol rank tolerance for last eigenvalue of 
 * @return refined Point3
 */
Point3 optimizeCustom(const NonlinearFactorGraph& graph,
    const Values& values, Key landmarkKey, double rank_tol);

Point3 triangulateNonlinearCustom(
    const std::vector<PinholeModel::shared_ptr>& intrinsics,
    const std::vector<Pose3>& poses,
    const Point2Vector& measurements,
    const Point3& initialEstimate,
    SharedNoiseModel noise_model,
    double rank_tol);

/**
 * Function to triangulate 3D landmark point from an arbitrary number
 * of poses (at least 2) using the DLT. This function is similar to the one
 * above, except that each camera has its own calibration. The function
 * checks that the resulting point lies in front of all cameras, but has
 * no other checks to verify the quality of the triangulation.
 * @param factors projection factor bundle
 * @param poses poses of cameras
 * @param measurements A vector of camera measurements
 * @param rank_tol rank tolerance, default 1e-9
 * @param optimize Flag to turn on nonlinear refinement of triangulation
 * @return Returns a Point3
 */
Point3 triangulatePoint3Custom(
    const std::vector<PinholeModel::shared_ptr>& intrinsics,
    const std::vector<Pose3>& poses,
    const Point2Vector& measurements, double rank_tol = 1e-9,
    bool optimize = false,
    SharedNoiseModel noise_model = noiseModel::Unit::Create(2));

/// triangulateSafeCustom: extensive checking of the outcome
TriangulationResult triangulateSafeCustom(
    const std::vector<PinholeModel::shared_ptr>& intrinsics,
    const std::vector<Pose3>& poses,
    const Point2Vector& measured,
    const TriangulationParameters& params,
    int min_obs,
    SharedNoiseModel noise_model);

size_t triangulateLandmarks(
    const std::vector<gtsam::Pose3>& extrinsics,
    const CameraObservations& obs_factors,
    const TriangulationParameters& triangulation_params,
    int min_obs,
    NonlinearFactorGraph& graph,
    Values& values);

} // end namespace gtsam