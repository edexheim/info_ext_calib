#include "calibration/backend/triangulation.h"
#include "calibration/factors/TriangulationFactorCustom.h"
#include <tbb/tbb_exception.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>

namespace gtsam {

Vector4 triangulateHomogeneousDLTCustom(
    const std::vector<Matrix34, Eigen::aligned_allocator<Matrix34>>& projection_matrices,
    const Point2Vector& measurements, double rank_tol) {

  // number of cameras
  size_t m = projection_matrices.size();

  // Allocate DLT matrix
  Matrix A = Matrix::Zero(m * 2, 4);

  for (size_t i = 0; i < m; i++) {
    size_t row = i * 2;
    const Matrix34& projection = projection_matrices.at(i);
    const Point2& p = measurements.at(i);

    // build system of equations

    // Normalized rows (gtsam version does not do this)
    Vector4 row1 = p.x() * projection.row(2) - projection.row(0);
    row1.normalize();
    Vector4 row2 = p.y() * projection.row(2) - projection.row(1);
    row2.normalize();
    A.row(row) = row1;
    A.row(row + 1) = row2;
  }
  int rank;
  double error;
  Vector v;
  boost::tie(rank, error, v) = DLT(A, rank_tol);

  if (rank < 3) {
    throw(TriangulationUnderconstrainedException());
  }

  return v;
}

Point3 triangulateDLTCustom(const std::vector<Matrix34, Eigen::aligned_allocator<Matrix34>>& projection_matrices,
    const Point2Vector& measurements, double rank_tol) {

  Vector4 v = triangulateHomogeneousDLTCustom(projection_matrices, measurements, rank_tol);

  // Create 3D point from homogeneous coordinates
  return Point3(v.head<3>() / v[3]);
}

std::pair<NonlinearFactorGraph, Values> triangulationGraphCustom(
    const std::vector<PinholeModel::shared_ptr>& intrinsics,
    const std::vector<Pose3>& poses,
    const Point2Vector& measurements,
    Key landmarkKey,
    const Point3& initialEstimate,
    SharedNoiseModel noise_model) {
  Values values;
  values.insert(landmarkKey, initialEstimate); // Initial landmark value
  NonlinearFactorGraph graph;
  // static SharedNoiseModel unit(noiseModel::Unit::Create(
  //     traits<Point2>::dimension));
  for (size_t i = 0; i < measurements.size(); i++) {
    graph.emplace_shared<TriangulationFactorCustom> //
        (measurements[i], poses[i], intrinsics[i], noise_model, landmarkKey);
  }
  return std::make_pair(graph, values);
}

Point3 optimizeCustom(const NonlinearFactorGraph& graph, const Values& values,
    Key landmarkKey, double rank_tol) {
  // Maybe we should consider Gauss-Newton?
  LevenbergMarquardtParams params;
  params.verbosityLM = LevenbergMarquardtParams::TRYLAMBDA;
  params.verbosity = NonlinearOptimizerParams::ERROR;
  params.lambdaInitial = 1;
  params.lambdaFactor = 10;
  params.maxIterations = 100;
  params.absoluteErrorTol = 1.0;
  params.verbosityLM = LevenbergMarquardtParams::SILENT;
  params.verbosity = NonlinearOptimizerParams::SILENT;
  params.linearSolverType = NonlinearOptimizerParams::MULTIFRONTAL_CHOLESKY;

  LevenbergMarquardtOptimizer optimizer(graph, values, params);
  Values result = optimizer.optimize();

  // Check rank tolerance at minimum
  Marginals marginals(graph, result);
  Matrix info = marginals.marginalInformation(landmarkKey);
  Eigen::JacobiSVD<Matrix> svd(info);
  Eigen::MatrixXd sigmas = svd.singularValues();
  double condition_number = sigmas(2)/sigmas(0);
  // std::cout << "Condition num: " << condition_number << std::endl;
  if (condition_number < rank_tol)
    throw(TriangulationUnderconstrainedException());

  return result.at<Point3>(landmarkKey);
}

Point3 triangulateNonlinearCustom(
    const std::vector<PinholeModel::shared_ptr>& intrinsics,
    const std::vector<Pose3>& poses,
    const Point2Vector& measurements,
    const Point3& initialEstimate,
    SharedNoiseModel noise_model,
    double rank_tol) {

  // Create a factor graph and initial values
  Values values;
  NonlinearFactorGraph graph;
  boost::tie(graph, values) = triangulationGraphCustom //
      (intrinsics, poses, measurements, Symbol('p', 0), initialEstimate, noise_model);

  // optimize in original GTSAM triangulation.cpp using LM optimization
  return optimizeCustom(graph, values, Symbol('p', 0), rank_tol);
}

Point3 triangulatePoint3Custom(
    const std::vector<PinholeModel::shared_ptr>& intrinsics,
    const std::vector<Pose3>& poses,
    const Point2Vector& measurements, double rank_tol,
    bool optimize,
    SharedNoiseModel noise_model) {

  size_t m = intrinsics.size();
  assert(measurements.size() == m);

  if (m < 2)
    throw(TriangulationUnderconstrainedException());

  // Undistort measurements (since CameraProjectionMatrix ignores them)
  Point2Vector measurements_calibrated;
  measurements_calibrated.reserve(m);
  for (int i=0; i<m; i++) {
    measurements_calibrated.push_back(
        intrinsics[i]->calibrate(measurements[i]));
  } 

  // construct projection matrices from poses & calibration
  std::vector<Matrix34, Eigen::aligned_allocator<Matrix34>> projection_matrices;
  projection_matrices.reserve(m);
  for(const Pose3& pose: poses) {
    // projection_matrices.push_back(
    //     CameraProjectionMatrix<typename CAMERA::CalibrationType>(camera.calibration())(
    //         camera.pose()));
    // Already calibrated points (so now just identity intrinsic matrix)
    // Pose3 pose = camera.pose();
    projection_matrices.push_back( 
        (pose.inverse().matrix()).block<3, 4>(0, 0) );
  }
  // Set DLT rank tolerance very low if optimizing, since we will check if well-constrained later
  double dlt_rank_tol = optimize ? 1e-9 : rank_tol;
  Point3 point = triangulateDLTCustom(projection_matrices, measurements_calibrated, dlt_rank_tol);

  // The n refine using non-linear optimization
  if (optimize) {
    try {
      point = triangulateNonlinearCustom(intrinsics, poses, measurements, point, noise_model, rank_tol);
    }
    // catch (const tbb::captured_exception& ex) {
    catch (const gtsam::IndeterminantLinearSystemException& ex) {
      throw(TriangulationUnderconstrainedException());
    }
  }

// #ifdef GTSAM_THROW_CHEIRALITY_EXCEPTION
  // verify that the triangulated point lies in front of all cameras
  for(const Pose3& pose: poses) {
    const Point3& p_local = pose.transformTo(point);
    if (p_local.z() <= 0) {
      throw(TriangulationCheiralityException());
    }
  }
// #endif

  return point;
}

/// triangulateSafeCustom: extensive checking of the outcome
TriangulationResult triangulateSafeCustom(
    const std::vector<PinholeModel::shared_ptr>& intrinsics,
    const std::vector<Pose3>& poses,
    const Point2Vector& measured,
    const TriangulationParameters& params,
    int min_obs,
    SharedNoiseModel noise_model) {

  size_t m = intrinsics.size();

  // if we have a single pose the corresponding factor is uninformative
  if (m < min_obs)
    return TriangulationResult::Degenerate();
  else
    // We triangulate the 3D position of the landmark
    try {
      Point3 point = triangulatePoint3Custom(intrinsics, poses, measured,
          params.rankTolerance, params.enableEPI);

      // Check landmark distance and re-projection errors to avoid outliers
      size_t i = 0;
      double maxReprojError = 0.0;
      for(const Pose3& pose: poses) {
        if (params.landmarkDistanceThreshold > 0
            && distance3(pose.translation(), point)
                > params.landmarkDistanceThreshold)
          return TriangulationResult::FarPoint();
// #ifdef GTSAM_THROW_CHEIRALITY_EXCEPTION
        // verify that the triangulated point lies in front of all cameras
        // Only needed if this was not yet handled by exception
        const Point3& p_local = pose.transformTo(point);
        if (p_local.z() <= 0)
          return TriangulationResult::BehindCamera();
// #endif
        // Check reprojection error
        if (params.dynamicOutlierRejectionThreshold > 0) {
          const Point2& zi = measured.at(i);
          Point2 hi = intrinsics[i]->uncalibratePixelDeriv(PinholeBase::Project(poses[i].transformTo(point)));
          Point2 reprojectionError(hi - zi);
          maxReprojError = std::max(maxReprojError, reprojectionError.norm());
        }
        i += 1;
      }

      // std::cout << maxReprojError << std::endl;
      // Flag as outlier if average reprojection error is too large
      if (params.dynamicOutlierRejectionThreshold > 0
          && maxReprojError > params.dynamicOutlierRejectionThreshold)
        return TriangulationResult::Outlier();

      // all good!
      return TriangulationResult(point);
    } catch (TriangulationUnderconstrainedException&) {
      // This exception is thrown if
      // 1) There is a single pose for triangulation - this should not happen because we checked the number of poses before
      // 2) The rank of the matrix used for triangulation is < 3: rotation-only, parallel cameras (or motion towards the landmark)
      // 3) EPI is enabled and optimization fails
      return TriangulationResult::Degenerate();
    } catch (TriangulationCheiralityException&) {
      // point is behind one of the cameras: can be the case of close-to-parallel cameras or may depend on outliers
      return TriangulationResult::BehindCamera();
    }
}

size_t triangulateLandmarks(
    const std::vector<gtsam::Pose3>& extrinsics,
    const CameraObservations& obs_factors,
    const TriangulationParameters& triangulation_params,
    int min_obs,
    NonlinearFactorGraph& graph,
    Values& values) {

  size_t num_landmarks = 0;

  // Adding factors to graph and initializing estimates
  for (const auto& [landmark_id, factors] : obs_factors) {
    // Add rekeyed factors to graph and setup cameras for triangulation
    std::vector<PinholeModel::shared_ptr> intrinsics;
    std::vector<Pose3> poses;
    Point2Vector measured;
    for (const auto& factor : factors) {
      intrinsics.emplace_back((factor->calibration()));
      const Pose3& pose = values.at<Pose3>(factor->key1());
      gtsam::Symbol ext_symbol = factor->key2();
      size_t cam_id = ext_symbol.index();
      const Pose3& ext = extrinsics[cam_id];
      poses.emplace_back(pose.compose(ext));
      measured.emplace_back(factor->measured());
    }
    // If successfully triangulated, add to graph
    // gtsam::Symbol(gtsam::LabeledSymbol('l', 'A', landmark_id)).print();
    TriangulationResult result = triangulateSafeCustom(
      intrinsics, poses, measured, triangulation_params, min_obs, factors[0]->noiseModel());
    if (result) {
      // If same key already exists from another motion partition
      // Ignore newer factors, since otherwise may constrain problem oddly
      // Each motion partition will have its own prior, which could disagree with projection factors
      // std::cout << result << std::endl;
      gtsam::Key landmark_key = factors[0]->key3();
      const auto& [it, was_inserted] = values.tryInsert(
          landmark_key,
          static_cast<const gtsam::Value&>(gtsam::GenericValue<gtsam::Point3>(*result)) );
      if (was_inserted) {
        graph.add(factors);
        num_landmarks++;
      }
    }
  }

  return num_landmarks;
}

} // end namespace gtsam