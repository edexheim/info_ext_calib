#include "calibration/backend/BatchSegmentDatabase.h"
#include "calibration/utils/gtsam_utils.h"
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Marginals.h>
#include "calibration/backend/partition.h"
#include "calibration/backend/triangulation.h"
#include "calibration/factors/PriorFactorTranslation.h"
#include "calibration/factors/epipolar/EpipolarFactor.h"

BatchSegmentDatabase::BatchSegmentDatabase(
    const BackendParams::SegmentDatabaseParams& database_params,
    const BackendParams::NoiseModelParams& noise_params,
    const gtsam::TriangulationParameters& triangulation_params)
  : SegmentDatabase(database_params, noise_params, triangulation_params),
    optimizer_params_(gtsam::LevenbergMarquardtParams::CeresDefaults())
    // optimizer_(std::make_unique<gtsam::LevenbergMarquardtOptimizer>(
        // gtsam::NonlinearFactorGraph(), gtsam::Values(), optimizer_params_)) // Will overwrite this later when segment arrives
{ 
  // optimizer_params_.verbosityLM = gtsam::LevenbergMarquardtParams::TRYLAMBDA;
  optimizer_ = std::make_unique<gtsam::LevenbergMarquardtOptimizer>(
        gtsam::NonlinearFactorGraph(), gtsam::Values(), optimizer_params_);
}

BatchSegmentDatabase::~BatchSegmentDatabase()
{
}

 std::vector<gtsam::Pose3> BatchSegmentDatabase::getExtrinsicsEstimates() const {
   return SegmentDatabase::getExtrinsicsEstimates(optimizer_->values());
 }

void BatchSegmentDatabase::retrieveEstimates(BackendOutput::shared_ptr& output_data) {
  gtsam::Values values = optimizer_->values();
  SegmentDatabase::retrieveEstimates(values, output_data);

  // Marginals
  // Negative 1s to indicate not in estimation?
  try {
    gtsam::Marginals marginals(optimizer_->graph(), values, gtsam::Marginals::Factorization::QR);
    for (size_t l = 0; l < num_cam_; l++) {
      try {
        gtsam::Matrix6 I = marginals.marginalInformation(TvcKey(l));
        output_data->marginal_information_.push_back(I);
      }
      catch (...) { // TODO: Specific types?
        output_data->marginal_information_.push_back(-1.0*gtsam::Matrix6::Identity());
      }
    }
  }
  catch (const tbb::captured_exception& ex) {
    for (size_t l = 0; l < num_cam_; l++)
      output_data->marginal_information_.push_back(-1.0*gtsam::Matrix6::Identity());
  }
}

bool BatchSegmentDatabase::optimizeStep() {
  // Taken mostly from NonlinearOptimizer defaultOptimize(), but only one step now

  if (!can_optimize_)
    return true;

  bool converged = false;

  const auto& params = optimizer_->params();
  double currentError = optimizer_->error();

  // check if we're already close enough
  if (currentError <= params.errorTol) {
    if (params.verbosity >= gtsam::NonlinearOptimizerParams::ERROR)
      cout << "Exiting, as error = " << currentError << " < " << params.errorTol << endl;
    converged = true;
    return converged;
  }

  // Maybe show output
  if (params.verbosity >= gtsam::NonlinearOptimizerParams::VALUES)
    optimizer_->values().print("Initial values");
  if (params.verbosity >= gtsam::NonlinearOptimizerParams::ERROR)
    cout << "Initial error: " << currentError << endl;

  // Iterative loop
  double newError = currentError; // used to avoid repeated calls to error()
  // Do next iteration
  currentError = newError;
  try {
    optimizer_->iterate();
  }
  catch (const tbb::captured_exception& ex) {
    // At this point, construct new graph in batch
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values values;
    this->constructGraphBatch(graph, values);
    // TODO: Take previous estimtes from optimizer as initial guess since will be better most likely
    optimizer_ = std::make_unique<gtsam::LevenbergMarquardtOptimizer>(graph, values, optimizer_params_);
  }

  // Update newError for either printouts or conditional-end checks:
  newError = optimizer_->error();

  // Maybe show output
  if (params.verbosity >= gtsam::NonlinearOptimizerParams::VALUES)
    optimizer_->values().print("newValues");
  if (params.verbosity >= gtsam::NonlinearOptimizerParams::ERROR)
    cout << "newError: " << newError << endl;

  // std::cout <<  currentError << " " << newError << std::endl;
  if (gtsam::checkConvergence(params.relativeErrorTol, params.absoluteErrorTol, params.errorTol,
      currentError, newError, params.verbosity) && std::isfinite(currentError)) {
    converged = true;
  }

  return converged;
}

void BatchSegmentDatabase::addCalibrationVariables(const std::vector<gtsam::Pose3>& extrinsics) {
  optimizer_ = std::make_unique<gtsam::LevenbergMarquardtOptimizer>(
        gtsam::NonlinearFactorGraph(), calib_values_init_, optimizer_params_);
}

void BatchSegmentDatabase::addSegment(const Segment& segment_to_add) {
  segments_.emplace(segment_to_add.getId(), segment_to_add);
}

void BatchSegmentDatabase::removeSegments(const std::set<SegmentId>& ids_to_remove) {
  // Remove segments
  for (const auto& id : ids_to_remove) {
    segments_.erase(id);
  }

  // At this point, construct new graph in batch
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;
  this->constructGraphBatch(graph, values);
  // TODO: Take previous estimtes from optimizer as initial guess since will be better most likely
  optimizer_ = std::make_unique<gtsam::LevenbergMarquardtOptimizer>(graph, values, optimizer_params_);
}

size_t BatchSegmentDatabase::constructGraphBatch(
    gtsam::NonlinearFactorGraph& graph,
    gtsam::Values& values) const {

  size_t tracked_landmarks = 0;
  size_t stereo_landmarks = 0;

  // Convert segments to independent partitions
  std::vector<Segment> segment_partitions = 
      partition::segmentsToPartitions(segments_, database_params_.landmark_overlap_thresh_);

  const gtsam::Values& opt_values = optimizer_->values();

  // 1. Add current estimate of calibration parameters
  std::vector<gtsam::Pose3> extrinsics;
  auto ext_t_sigma = gtsam::noiseModel::Isotropic::Sigma(3, 
        noise_params_.extrinsic_translation_sigma_);
  for (int l=0; l<num_cam_; l++) {
    const gtsam::Pose3& ext = opt_values.at(TvcKey(l)).cast<gtsam::Pose3>();
    values.insert(TvcKey(l), ext);
    extrinsics.push_back(ext);
    
    // Add extrinsic translation prior
    auto ext_init = calib_values_init_.at<gtsam::Pose3>(TvcKey(l));
    auto ext_t_prior = boost::make_shared<gtsam::PriorFactorTranslation>(
        TvcKey(l), ext_init.translation(), ext_t_sigma);
    graph.add(ext_t_prior);

  }
  char Tvc_char = TvcKey(0).chr();

  // 2. Insert values, rekey factors, and insert into graph
  size_t prev_sess_id(std::numeric_limits<size_t>::max());
  std::map<gtsam::Key, gtsam::Key> rekey_map;
  // TODO: Only limited number of unique sessions allowed? ASCII
  // unsigned char sess_char = 'A'-1;
  for (size_t i = 0; i < segment_partitions.size(); i++) {
    const auto& segment = segment_partitions[i];
    // if new session, need to change keys
    if (segment.getSessionId() != prev_sess_id) {
      rekey_map.clear();
      // sess_char++;
    }

    unsigned sess_char = this->getSessionChar(segment.getSessionId());

    // 2.a. Values

    // Add vehicle poses to values and rekey_map
    const gtsam::Values& seg_values = segment.getValues();
    for(const auto& key_value : seg_values) {
      gtsam::Symbol old_symbol = key_value.key; // Non-const duplicate to deal with non-const insert argument
      // Do not insert calibration vars
      if (old_symbol.chr() != Tvc_char) {
        // Generate new symbol with session id
        gtsam::LabeledSymbol new_symbol(old_symbol.chr(), sess_char, old_symbol.index());
        values.insert(new_symbol, key_value.value);
        // Track in key remapping
        rekey_map.insert({old_symbol, new_symbol});
      }
    }

    // Add landmark keys to rekey_map (Values come later)
    const CameraObservations obs_factors = segment.getObsGraph();
    for (const auto& [landmark_id, factors] : obs_factors) {
      for (const auto& factor : factors) {
        const auto& [it, inserted] = rekey_map.insert({factor->key3(), 
            gtsam::LabeledSymbol('l', sess_char, landmark_id) });
      }
    }

    const CameraObservations stereo_obs = segment.getStereoObs();
    for (const auto& [landmark_id, factors] : stereo_obs) {
      for (const auto& factor : factors) {
        const auto& [it, inserted] = rekey_map.insert({factor->key3(), 
            gtsam::LabeledSymbol('s', sess_char, landmark_id) });
      }
    }

    // 2.b. Factors

    // Omit first odometry (disconnected or dummy for first sequence)
    const gtsam::NonlinearFactorGraph& odom_chain = segment.getOdomGraph();
    gtsam::NonlinearFactorGraph rekeyed_odom_graph = odom_chain.rekey(rekey_map);
    graph.push_back(rekeyed_odom_graph.begin()+1, rekeyed_odom_graph.end());

    // Add first pose prior
    size_t first_id = segment.getFirstId();
    auto prior_noise_model = gtsam::noiseModel::Isotropic::Sigma(6, 
        noise_params_.pose_prior_sigma_);
    auto prior_factor = boost::make_shared<gtsam::PriorFactor<gtsam::Pose3> >
      (gtsam::PriorFactor<gtsam::Pose3>(rekey_map[TwvKey(first_id)], 
                                        seg_values.at<gtsam::Pose3>(TwvKey(first_id)), 
                                        prior_noise_model));
    graph.add(prior_factor);


    // Merge projection factors
    // gtsam::Symbol (*landmark_key_func)(size_t l) = lKey;
    // CameraObservations merged_obs;
    // graph_utils::mergeProjectionFactors(obs_factors, segment.getFeatureIdMatches(), 
    //     landmark_key_func,
    //     merged_obs);
    // // Rekey projection factors
    CameraObservations rekeyed_obs;
    for (const auto& [landmark_id, landmark_obs] : obs_factors) {
      for (const auto& factor : landmark_obs) {
        rekeyed_obs[landmark_id].push_back(
            boost::dynamic_pointer_cast<ProjectionFactor>(
                factor->rekey(rekey_map) ) );
      }
    }
    // Rekey stereo-only projection factors
    CameraObservations stereo_rekeyed_obs;
    for (const auto& [landmark_id, landmark_obs] : stereo_obs) {
      for (const auto& factor : landmark_obs) {
        stereo_rekeyed_obs[landmark_id].push_back(
            boost::dynamic_pointer_cast<ProjectionFactor>(
                factor->rekey(rekey_map) ) );
      }
    }

    // std::cout << "Temporal landmarks" << std::endl;

    // Add projection factors for successfully triangulated landmarks    
    tracked_landmarks += gtsam::triangulateLandmarks(
        extrinsics, rekeyed_obs, triangulation_params_, 2,
        graph, values);

    // std::cout << "Stereo landmarks" << std::endl;

    // Add stereo factors (projection or epipolar) 
    if (!noise_params_.stereo_use_epipolar_) {
      stereo_landmarks += gtsam::triangulateLandmarks(
          extrinsics, stereo_rekeyed_obs, triangulation_params_, 2,
          graph, values);
    }
    else {
      auto gaussian_model = gtsam::noiseModel::Isotropic::Sigma(1, noise_params_.epipolar_sigma_);
      auto epi_noise_model = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(1.345), gaussian_model);
      for (const auto& [id, proj_factors] : stereo_rekeyed_obs) {
        const auto& intrinsics0 = proj_factors[0]->calibration();
        const gtsam::Point2& obs0 = proj_factors[0]->measured();
        gtsam::Point2 obs0_undistorted = intrinsics0->calibrate(obs0);
        gtsam::Point3 r0(obs0_undistorted(0), obs0_undistorted(1), 1.0);
        gtsam::Key key0 = proj_factors[0]->key2();

        const auto& intrinsics1 = proj_factors[1]->calibration();
        const gtsam::Point2& obs1 = proj_factors[1]->measured();
        gtsam::Point2 obs1_undistorted = intrinsics1->calibrate(obs1);
        gtsam::Point3 r1(obs1_undistorted(0), obs1_undistorted(1), 1.0);
        gtsam::Key key1 = proj_factors[1]->key2();

        auto epi_factor = boost::make_shared<gtsam::EpipolarFactor>(
            key0, key1, r0, r1, 
            noise_params_.epipolar_use_sampson_, epi_noise_model);
        graph.push_back(epi_factor);
      }
    }

    prev_sess_id = segment.getSessionId();
  }

  // std::cout << "Tracked landmarks: " << tracked_landmarks << " stereo landmarks: " << stereo_landmarks << std::endl;
  size_t num_landmarks = tracked_landmarks + stereo_landmarks;
  return num_landmarks;
}