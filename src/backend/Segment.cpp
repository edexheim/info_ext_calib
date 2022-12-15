#include "calibration/backend/Segment.h"
#include "calibration/utils/gtsam_utils.h"
#include "calibration/utils/info_metrics.h"
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/linearExceptions.h>
#include "calibration/backend/graph_utils.h"
#include "calibration/utils/gtsam_utils.h"
#include "calibration/backend/triangulation.h"
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

Segment::Segment( const gtsam::NonlinearFactorGraph& graph_odom,
                  const CameraObservations& graph_obs,
                  const CameraObservations& stereo_obs,
                  const FeatureIdMatches& feature_id_matches,
                  const gtsam::Values& values,
                  size_t first_id)
                  : graph_odom_(graph_odom), 
                    graph_obs_(graph_obs), 
                    stereo_obs_(stereo_obs),
                    feature_id_matches_(feature_id_matches),
                    values_(values), 
                    first_id_(first_id),
                    has_id_(false), id_(), has_info_(false)
{

}

Segment::~Segment() {
  
}

void Segment::calcInfoContent(
    const std::vector<gtsam::Pose3>& extrinsics,
    const BackendParams::NoiseModelParams& noise_params,
    const gtsam::TriangulationParameters& triangulation_params,
    const BackendParams::MarginalCovarianceParams& marg_cov_params) {

  uint8_t NUM_CAM = extrinsics.size();

  // gttic_(Segment_calcInfoContent);

  // Create graph with prior to constrain system
  gtsam::NonlinearFactorGraph graph_with_prior;

  // Add current extrinsics estimate to values
  gtsam::Values values = values_;
  for (int l=0; l<extrinsics.size(); l++) {
    values.insert(TvcKey(l), extrinsics[l]);
    // Add extremely loose priors to avoid ColamdConstrained bugs when no extrinsics factors
    // This happens when no landmarks can be triangulated
    graph_with_prior.addPrior(TvcKey(l), extrinsics[l], 
        gtsam::noiseModel::Isotropic::Precision(6, 1e-9));
  }

  // Odometry (omit first since connects to previous segment or dummy)
  graph_with_prior.push_back(graph_odom_.begin()+1, graph_odom_.end());

  // Prior on first pose
  gtsam::noiseModel::Isotropic::shared_ptr noise_model = 
      gtsam::noiseModel::Isotropic::Sigma(6, 
          noise_params.pose_prior_sigma_);
  gtsam::PriorFactor<gtsam::Pose3>::shared_ptr prior_factor = 
    boost::make_shared<gtsam::PriorFactor<gtsam::Pose3> >
      (gtsam::PriorFactor<gtsam::Pose3>( TwvKey(first_id_), 
                                        values.at<gtsam::Pose3>(TwvKey(first_id_)), 
                                        noise_model));
  graph_with_prior.add(prior_factor);

  
  // Merge projection factors
  gtsam::Symbol (*landmark_key_func)(uint64_t id) = lKey;
  CameraObservations merged_obs;
  graph_utils::mergeProjectionFactors(graph_obs_, feature_id_matches_, 
      landmark_key_func,
      merged_obs);

  // Triangulate landmarks and add projection factors
  size_t num_landmarks = gtsam::triangulateLandmarks(
          extrinsics, merged_obs, 
          triangulation_params, 2,
          graph_with_prior, values);

  // Stereo only factors   
  num_landmarks += gtsam::triangulateLandmarks(
      extrinsics, stereo_obs_, 
      triangulation_params, 2,
      graph_with_prior, values);
  
  
  // Construct variable index
  gtsam::VariableIndex var_index(graph_with_prior);
  
  // Constrained ordering should improve efficiency to get calibration marginals
  gtsam::Ordering ordering = this->getOrdering(var_index, NUM_CAM);


  // Recover extrinsics marginal covariance
  calib_info_.resize(2*NUM_CAM, std::numeric_limits<double>::max());
  marg_info_.resize(2*NUM_CAM, gtsam::Matrix3::Zero());
  try {
    gtsam::LevenbergMarquardtOptimizer optimizer(graph_with_prior, values, gtsam::LevenbergMarquardtParams::CeresDefaults());
    gtsam::Values opt_values = optimizer.optimize();

    gtsam::Marginals marginals( graph_with_prior, 
                                opt_values,
                                // TODO: Use QR or Cholesky?
                                gtsam::Marginals::Factorization::QR,
                                ordering);
    

    gtsam::Matrix3 cov_R = marg_cov_params.rotation_sigma_ * gtsam::Matrix3::Identity();
    gtsam::Matrix3 cov_R_inv = cov_R.inverse();
    gtsam::Matrix3 cov_t = marg_cov_params.translation_sigma_ * gtsam::Matrix3::Identity();
    gtsam::Matrix3 cov_t_inv = cov_t.inverse();

    for (int l=0; l<NUM_CAM; l++) {
      int ind_info_R = 2*l;
      int ind_info_t = ind_info_R + 1;

      // Obtain Jacobians for SE(3) -> SO(3) x R^3 for covariance propagation
      //   using extrinsics linearization point
      gtsam::Matrix6 E_se3 = marginals.marginalCovariance(TvcKey(l));
      gtsam::Matrix36 J_so3_se3;
      const gtsam::Rot3& R = extrinsics[l].rotation(J_so3_se3);
      gtsam::Matrix3 E_R = J_so3_se3 * E_se3 * J_so3_se3.transpose();
      gtsam::Matrix3 E_R_norm = cov_R_inv * E_R * cov_R_inv;

      gtsam::Matrix36 J_r3_se3;
      const gtsam::Point3& t = extrinsics[l].translation(J_r3_se3);
      gtsam::Matrix3 E_t = J_r3_se3 * E_se3 * J_r3_se3.transpose();
      gtsam::Matrix3 E_t_norm = cov_t_inv * E_t * cov_t_inv;

      // Extract covariance blocks for rotation and translation
      marg_info_[ind_info_R] = E_R_norm.inverse();
      marg_info_[ind_info_t] = E_t_norm.inverse();
      calib_info_[ind_info_R] = info_metrics::dOpt(E_R_norm);
      calib_info_[ind_info_t] = info_metrics::dOpt(E_t_norm);
      std::cout << "Info content: " << std::scientific << calib_info_[ind_info_R] << " " << calib_info_[ind_info_t]  << std::endl;
    }
  }
  catch (const tbb::captured_exception& ex) {
    std::cout << "Bad marginals: setting segment covariance to max" << std::endl;
    for (int l=0; l<NUM_CAM; l++) {
      int ind_info_R = 2*l;
      int ind_info_t = ind_info_R + 1;
      marg_info_[ind_info_R] = gtsam::Matrix3::Zero();
      marg_info_[ind_info_t] = gtsam::Matrix3::Zero();
      calib_info_[ind_info_R] = std::numeric_limits<double>::max();
      calib_info_[ind_info_t] = std::numeric_limits<double>::max();
    }
  }

  has_info_ = true;
}

gtsam::Ordering Segment::getOrdering(const gtsam::VariableIndex& var_index, uint8_t num_cam) const {
  // ordering - force calib params last
  gtsam::KeyVector key_vec;
  for (int l=0; l<num_cam; l++) {
    key_vec.push_back(TvcKey(l));
  }
  bool force_order = true; // extrinsics eliminated in ascending order
  return gtsam::Ordering::ColamdConstrainedLast(var_index, key_vec, force_order);
}