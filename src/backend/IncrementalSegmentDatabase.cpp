#include "calibration/backend/IncrementalSegmentDatabase.h"
#include "calibration/utils/gtsam_utils.h"
#include "calibration/backend/triangulation.h"
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearOptimizer.h>
#include "calibration/factors/PriorFactorTranslation.h"


IncrementalSegmentDatabase::IncrementalSegmentDatabase(
    const BackendParams::SegmentDatabaseParams& database_params,
    const BackendParams::NoiseModelParams& noise_params,
    const gtsam::TriangulationParameters& triangulation_params,
    const gtsam::FastMap<char, gtsam::Vector> relin_params)
  : SegmentDatabase(database_params, noise_params, triangulation_params),
    isam2_(gtsam::ISAM2Params(
        // gtsam::ISAM2DoglegParams(),
        gtsam::ISAM2DoglegParams(1.0, 1e-5, gtsam::DoglegOptimizerImpl::ONE_STEP_PER_ITERATION, false),
        // gtsam::ISAM2GaussNewtonParams(),
        1e-6 /*relin_params*/, 1, true, true, gtsam::ISAM2Params::CHOLESKY)),
    curr_factor_index_(0)
{
  
  // TODO: Avoid having number of factors grow unbounded
  // How to account for new slots though?
  // isam2_.params_.findUnusedFactorSlots = false;

  // Relinearization thresholds
  // isam2_.params_.relinearizeThreshold = relin_thresholds;
  // for (const auto& [c, v] : relin_params)
    // std::cout << c << " " << v.transpose() << std::endl;
}

IncrementalSegmentDatabase::~IncrementalSegmentDatabase() {
}

void IncrementalSegmentDatabase::addCalibrationVariables(const std::vector<gtsam::Pose3>& extrinsics) {
  // isam2_.update(gtsam::NonlinearFactorGraph(), calib_values_init_);
}

bool IncrementalSegmentDatabase::addToPartitions(
    const Segment& segment_to_add, bool& consecutive) {

  // std::cout << "ADDING SEGMENT TO PARTITION" << std::endl;

  bool new_partition = false;
  SegmentId new_seg_id = segment_to_add.getId();
  consecutive = false;
  if (!segments_.empty()) {
    SegmentId last_seg_id = segments_.rbegin()->first;

    // If consecutive, add to most recent partition
    if (new_seg_id.isSequential(last_seg_id)) {
      segment_partitions_.back().insert(new_seg_id);
      consecutive = true;
    }
    // Otherwise, check landmark overlap with last segment (in same session)
    else if (new_seg_id.sess_id_ == last_seg_id.sess_id_) {
      const Segment& last_segment = segments_.at(last_seg_id);
      if (this->isEnoughLandmarkOverlap(last_segment, segment_to_add)) {
        segment_partitions_.back().insert(new_seg_id);
      }
      // Start new segment partition
      else {
        segment_partitions_.push_back({new_seg_id});
        new_partition = true;
      }
    }
    else {
      segment_partitions_.push_back({new_seg_id});
      new_partition = true;
    }
  }
  else {
    segment_partitions_.push_back({new_seg_id});
    new_partition = true;
  }
  // Store segment
  segments_.emplace(new_seg_id, segment_to_add);

  return new_partition;
}

void IncrementalSegmentDatabase::removeSegments(const std::set<SegmentId>& ids_to_remove) {

  gtsam::FactorIndices factors_to_remove;
  gtsam::NonlinearFactorGraph new_priors;

  // std::cout << "REMOVING SEGMENTS" << std::endl;
  // for (const auto& id : ids_to_remove)
    // std::cout << id << std::endl;

  // Step 1: Remove landmark indices

  // std::cout << "Step 1a: Remove landmark indices" << std::endl;  

  std::unordered_set<gtsam::Key> updated_landmarks;
  // First remove observatiosn and note which landmarks lost factors
  for (const auto& seg_id : ids_to_remove) {
    unsigned char sess_char = getSessionChar(seg_id.sess_id_);
    const Segment& segment = segments_.at(seg_id);
    const auto& seg_obs = segment.getObsGraph();
    // std::cout << seg_id << " Max landmarks: " << seg_obs.size() << std::endl;
    for (const auto& [landmark_id, factors] : seg_obs) {
      gtsam::Symbol old_symbol = factors[0]->key3();
      gtsam::LabeledSymbol new_symbol(old_symbol.chr(), sess_char, old_symbol.index());
      const auto& landmark_map_it = landmark_factor_indices_.find(new_symbol);
      // Landmark key only exists if landmark was triangulated
      if (landmark_map_it != landmark_factor_indices_.end()) {
        auto& segment_indices_map = landmark_map_it->second;

        // Note factor indices to be removed from ISAM2, and delete them from map
        const auto& indices_it = segment_indices_map.find(seg_id);
        // It is posssible for landmark to have been triangulated in different partition
        if (indices_it != segment_indices_map.end()) {
          factors_to_remove.insert(
              factors_to_remove.end(),
              indices_it->second.begin(),
              indices_it->second.end() );
          segment_indices_map.erase(indices_it);

          updated_landmarks.insert(new_symbol);
        }
      }
    }
  }

  // std::cout << "Num updated landmarks: " << updated_landmarks.size() << std::endl;
  // std::cout << "Num landmark factors to remove: " << factors_to_remove.size() << std::endl;

  // std::cout << "Step 1b: Check retriangulation of affected landmarks" << std::endl;
  
  // Go through updated landmarks and try to triangulate with remaining obs
  //    Otherwise remove all observations to that landmark
  // Get current isam2 values and factors for triangulation
  gtsam::Values isam2_values = isam2_.calculateBestEstimate();
  gtsam::NonlinearFactorGraph isam2_factors = isam2_.getFactorsUnsafe();
  for (const gtsam::Key& key : updated_landmarks) {
    const auto& segment_indices_map_it = landmark_factor_indices_.find(key);

    // TODO: More duplicate triangulation code...
    std::vector<gtsam::PinholeModel::shared_ptr> intrinsics;
    std::vector<gtsam::Pose3> poses;
    gtsam::Point2Vector measured;
    gtsam::SharedNoiseModel noise_model = nullptr;
    for (const auto& [seg_id, factor_indices] : segment_indices_map_it->second) {
      for (const auto& f_ind : factor_indices) {
        const auto& nonlinear_factor = isam2_factors.at(f_ind);
        ProjectionFactor::shared_ptr f = boost::dynamic_pointer_cast<ProjectionFactor>(nonlinear_factor);
        // std::cout << "pointer conversion successful? " << (f != nullptr) << std::endl;
        intrinsics.emplace_back((f->calibration()));
        const gtsam::Pose3& pose = isam2_values.at<gtsam::Pose3>(f->key1());
        const gtsam::Pose3& ext = isam2_values.at<gtsam::Pose3>(f->key2());
        poses.emplace_back(pose.compose(ext));
        measured.emplace_back(f->measured());
        noise_model = f->noiseModel();
      }
    }
    gtsam::TriangulationResult result = gtsam::triangulateSafeCustom(
        intrinsics, poses, measured, triangulation_params_, 3, noise_model);
    // If cannot triangulate, remove all indices to this landmark?
    // TODO TODO: Note new segment should be added before removing,
    //   otherwise this step may produce different results (less landmarks)
    if (!result) {
    // if (true) {
      for (const auto& [seg_id, factor_indices] : segment_indices_map_it->second) {
        // std::cout << seg_id << std::endl;
        factors_to_remove.insert(
          factors_to_remove.end(),
          factor_indices.begin(),
          factor_indices.end() );

      }
      // Delete landmark key-value pair from map
      landmark_factor_indices_.erase(segment_indices_map_it);
    }

  }

  // Step 2: Adjust partition bookkeeping

  // Only should add 1 prior per segment
  // ISAM2 variable index throws error if trying to remove factor before added
  // std::vector<bool> added_prior_to_partition(segment_partitions_.size(), false);

  // std::cout << "Step 2: Adjust partition bookkeeping" << std::endl;
  for (const auto& seg_id : ids_to_remove) {
    // TODO: Better way than checking all partitions for the removed segment?
    for (int i = 0; i < segment_partitions_.size(); i++) {
      auto& partition = segment_partitions_.at(i);
      const auto& id_it = partition.find(seg_id);
      if (id_it != partition.end()) {
        const auto& segment_it = segments_.find(*id_it);
        const Segment& segment = segment_it->second;

        // Get factor indices to remove from isam2
        const auto& factor_it = segment_factor_indices_.find(*id_it);
        const gtsam::FactorIndices& factor_indices = factor_it->second;
        factors_to_remove.insert(
            factors_to_remove.end(),
            factor_indices.begin(),
            factor_indices.end() );


        // If only segment of partition, erase entire partition
        if (partition.size() == 1) {
          segment_partitions_.erase(segment_partitions_.begin() + i);
          factors_to_remove.push_back(partition_priors_[i]);
          partition_priors_.erase(partition_priors_.begin() + i);
        }
        else {
          // Do not erase segment if entire partition deleted
          bool erase_segment = true;
          // If first segment of partition and others remain
          if (id_it == partition.begin()) {
            const SegmentId next_segment_id = *(std::next(id_it,1));
            // Check if next_segment_id is not going to be deleted anyway
            // Otherwise factor indexing will get messed up
            if (ids_to_remove.find(next_segment_id) == ids_to_remove.end()) {
              // std::cout << "REMOVING FIRST SEGMENT OF PARTITION" << std::endl;
              // Add prior to next segment
              const Segment& next_segment = segments_.at(next_segment_id);
              size_t first_id = next_segment.getFirstId();
              unsigned char sess_char = getSessionChar(next_segment_id.sess_id_);
              auto prior_noise_model = gtsam::noiseModel::Isotropic::Sigma(6, 
                  noise_params_.pose_prior_sigma_);
              // TODO: Should initial prior use original segment value 
              //        or current ISAM2 estimate? 
              auto prior_factor = this->createPriorFactor(first_id, sess_char, isam2_values);
              new_priors.add(prior_factor);
              factors_to_remove.push_back(partition_priors_[i]);
              partition_priors_[i] = curr_factor_index_;
              curr_factor_index_++;
            }
          }
          // If last segment of partition, just remove it
          else if ( std::next(id_it) == partition.end()) {

          }
          // If segments on both sides of segment to be removed,
          //   try to join them via landmarks 
          // Otherwise, split partition
          else {
            const SegmentId prev_segment_id = *(std::prev(id_it, 1));
            const SegmentId next_segment_id = *(std::next(id_it, 1));
            // Only attempt joining/splitting if both IDs will continue to be valid
            if ( ids_to_remove.find(prev_segment_id) == ids_to_remove.end()
                && ids_to_remove.find(next_segment_id) == ids_to_remove.end() ) { 

              // std::cout << "SPLITTING PARTITION" << std::endl;
              // std::cout << prev_segment_id << " " << next_segment_id << std::endl;

              const Segment& prev_segment = segments_.at(prev_segment_id);
              const Segment& next_segment = segments_.at(next_segment_id);

              // Split partition if not enough common observations
              // TODO TODO: WHAT TO DO WITH COMMON LANDMARKS IF PARTITION SPLIT
              if (!this->isEnoughLandmarkOverlap(prev_segment, next_segment)) {

                std::set<SegmentId> partition1, partition2;
                for (const auto& id : partition) {
                  if (id <= prev_segment_id) {
                    partition1.insert(id);
                  }
                  else if (id >= next_segment_id) {
                    partition2.insert(id);
                  }
                }
                // Add prior to first segment of each new partition


                // Erase old partition and add in the separated ones
                segment_partitions_.erase(segment_partitions_.begin() + i);
                segment_partitions_.insert(
                    segment_partitions_.begin() + i, partition1);
                segment_partitions_.insert(
                    segment_partitions_.begin() + 1 + i, partition2);

                auto prior_factor2 = this->createPriorFactor(
                    next_segment.getFirstId(), 
                    getSessionChar(next_segment_id.sess_id_),
                    isam2_values);
                new_priors.add(prior_factor2);
                partition_priors_.insert(
                    partition_priors_.begin() + i + 1, curr_factor_index_);
                curr_factor_index_++;
                
                // Entire partition reorganized, so do not still delete segment
                erase_segment = false;
              }
            }
          }

          // Erase segment from partition
          if (erase_segment) {
            partition.erase(id_it);
          }
        }

        segments_.erase(segment_it);
        segment_factor_indices_.erase(factor_it);

        // Break since found segment in this partition
        break;
      }
    }
  }

  // Update isam2

  gtsam::ISAM2Result isam2_result_remove = isam2_.update(
      new_priors, gtsam::Values(), 
      factors_to_remove, boost::none, calib_keys_);
}

void IncrementalSegmentDatabase::addSegment(const Segment& segment_to_add) {

  // std::cout << "ADDING SEGMENT" << std::endl;

  gtsam::NonlinearFactorGraph new_factors;
  gtsam::Values new_values;

  // std::cout << segment_to_add.getId() << std::endl;

  SegmentId new_seg_id = segment_to_add.getId();

  std::set<gtsam::Key> relevant_calib_keys;

  // Step 1: Update segment organization
  bool consecutive = false;
  bool new_partition = this->addToPartitions(segment_to_add, consecutive);

  // Step 2: Values
  // Insert new segment values and setup rekey map
  unsigned char sess_char = getSessionChar(new_seg_id.sess_id_);
  const gtsam::Values& segment_values = segment_to_add.getValues();
  for (const auto& [old_key, value] : segment_values) {
    gtsam::Symbol old_symbol = old_key;
    gtsam::LabeledSymbol new_symbol(old_symbol.chr(), sess_char, old_symbol.index());
    rekey_map_.insert({old_key, new_symbol});
    new_values.insert(new_symbol, value);
  }
  // Get current isam2 values for initializing landmarks
  gtsam::Values isam2_values = isam2_.calculateBestEstimate();

  gtsam::Values calib_values;
  for (const auto& [key, value] : calib_values_init_) {
    if (!isam2_values.exists(key)) {
      calib_values.insert(key, value);
    }
    else {
      calib_values.insert(key, isam2_values.at(key));
    }
  }

  // Step 3: Factor handling
  auto& segment_factor_ind = segment_factor_indices_[new_seg_id];

  // Prior handling
  if (new_partition) {
    size_t first_id = segment_to_add.getFirstId();

    auto prior_factor = this->createPriorFactor(
        first_id, 
        sess_char,
        new_values);
    new_factors.add(prior_factor);
    partition_priors_.push_back(curr_factor_index_);
    curr_factor_index_++;
  }

  // Odometry handling
  const auto& odom_factors = segment_to_add.getOdomGraph();
  // Add first odom to previous segment if consectuive
  if (consecutive) {
    // Add connecting odom for sequential segmentws
    new_factors.push_back(odom_factors[0]->rekey(rekey_map_));
    // Add factor index to both segments, since shared between
    SegmentId prev_seg_id(new_seg_id.sess_id_, new_seg_id.seq_id_-1); 
    segment_factor_indices_[prev_seg_id].push_back(curr_factor_index_);
    segment_factor_ind.push_back(curr_factor_index_);
    curr_factor_index_++;
  }
  // Add the rest of odom
  for (int i = 1; i < odom_factors.size(); i++) {
    new_factors.push_back(odom_factors[i]->rekey(rekey_map_));
    segment_factor_ind.push_back(curr_factor_index_);
    curr_factor_index_++;
  }
  
  // Stereo obs only depend on current segment, so either add or don't
  const auto& stereo_factors = segment_to_add.getStereoObs();
  for (const auto& [landmark_id, factors] : stereo_factors) {
    // TODO: This code duplicated a lot!
    std::vector<gtsam::PinholeModel::shared_ptr> intrinsics;
    std::vector<gtsam::Pose3> poses;
    gtsam::Point2Vector measured;
    gtsam::SharedNoiseModel noise_model = nullptr;
    for (const auto& f : factors) {
      intrinsics.emplace_back((f->calibration()));
      const gtsam::Pose3& pose = new_values.at<gtsam::Pose3>(rekey_map_.at(f->key1()));
      const gtsam::Pose3& ext = calib_values.at<gtsam::Pose3>(f->key2());
      poses.emplace_back(pose.compose(ext));
      measured.emplace_back(f->measured());
      noise_model = f->noiseModel();
    }
    gtsam::TriangulationResult result = gtsam::triangulateSafeCustom(
          intrinsics, poses, measured, triangulation_params_, 2, noise_model);
    if (result) {
      gtsam::Symbol old_symbol = factors[0]->key3();
      gtsam::Key new_key = gtsam::LabeledSymbol(old_symbol.chr(), sess_char, old_symbol.index());
      rekey_map_.insert({factors[0]->key3(), new_key});
      new_values.insert(new_key, *result);
      for (const auto& f : factors) {
        new_factors.push_back(f->rekey(rekey_map_));
        segment_factor_ind.push_back(curr_factor_index_);
        // No landmark factor ind since dependent on current segment
        curr_factor_index_++;

        relevant_calib_keys.insert(f->key2());
      }
    }
  }

  // std::cout << "Number of segment factors: " << segment_factor_ind.size() << std::endl;
  // std::cout << "New factors size before landmarks: " << new_factors.size() << std::endl;
  // std::cout << "Sess char " << sess_char << std::endl;

  // Check if tracked points can be triangulated using currrent ISAM2 estimates
  const auto& obs_factors = segment_to_add.getObsGraph();
  for (const auto& [landmark_id, factors] : obs_factors) {
    gtsam::Symbol old_landmark_symbol = factors[0]->key3();
    gtsam::Key new_key = gtsam::LabeledSymbol(old_landmark_symbol.chr(), sess_char, old_landmark_symbol.index());
    // If values does not already exist, attempt triangulation
    if (!isam2_.valueExists(new_key)) {
      // TODO: Gather observations from all segments which dictates
      //   new factors to be added to segment_factor_indices_...
      ProjectionFactorBundle proj_factors;
      std::vector<gtsam::PinholeModel::shared_ptr> intrinsics;
      std::vector<gtsam::Pose3> poses;
      gtsam::Point2Vector measured;
      gtsam::SharedNoiseModel noise_model = nullptr;
      std::vector<SegmentId> factor_segment_ids;

      // Handle new segment obs (Need to use new_values for poses)
      for (const auto& f : factors) {
        proj_factors.emplace_back(f);
        intrinsics.emplace_back((f->calibration()));
        const gtsam::Pose3& pose = new_values.at<gtsam::Pose3>(rekey_map_.at(f->key1()));
        const gtsam::Pose3& ext = calib_values.at<gtsam::Pose3>(f->key2());
        poses.emplace_back(pose.compose(ext));
        measured.emplace_back(f->measured());
        noise_model = f->noiseModel();
        factor_segment_ids.push_back(new_seg_id);
      }

      // Check all segments from the partition for observations
      for (const SegmentId& seg_id : segment_partitions_.back()) {
        // Do not check new segment since addded already
        if (seg_id == new_seg_id) 
          continue;
          
        const Segment& segment = segments_.at(seg_id);
        const auto& segment_obs = segment.getObsGraph();
        auto it = segment_obs.find(landmark_id);
        if (it != segment_obs.end()) {
          const auto& prev_factors = it->second;
          // Old factors should have pose estimates already from ISAM2
          for (const auto& f : prev_factors) {
            proj_factors.emplace_back(f);
            intrinsics.emplace_back((f->calibration()));
            // Note that this partition should have same session id
            gtsam::Symbol old_pose_symbol(f->key1());
            gtsam::Key pose_key = gtsam::LabeledSymbol(old_pose_symbol.chr(), sess_char, old_pose_symbol.index());
            const gtsam::Pose3& pose = isam2_values.at<gtsam::Pose3>(pose_key);
            const gtsam::Pose3& ext = calib_values.at<gtsam::Pose3>(f->key2());
            poses.emplace_back(pose.compose(ext));
            measured.emplace_back(f->measured());
            noise_model = f->noiseModel();
            factor_segment_ids.push_back(seg_id);
          }
        }
      }

      // Attempt triangulation
      gtsam::TriangulationResult result = gtsam::triangulateSafeCustom(
          intrinsics, poses, measured, triangulation_params_, 3, noise_model);
      if (result) {
        rekey_map_.insert({factors[0]->key3(), new_key});
        new_values.insert(new_key, *result);
        for (int i = 0; i < proj_factors.size(); i++) {
          const auto& f = proj_factors[i];
          new_factors.push_back(f->rekey(rekey_map_));
          const SegmentId& seg_id = factor_segment_ids[i];
          landmark_factor_indices_[new_key][seg_id].push_back(curr_factor_index_);
          curr_factor_index_++;

          relevant_calib_keys.insert(f->key2());
        }
      }
    }
    // Landmark already exists in optimization, so just add factors
    else {
      for (const auto& f : factors) {
        new_factors.push_back(f->rekey(rekey_map_));
        landmark_factor_indices_[new_key][new_seg_id].push_back(curr_factor_index_);
        curr_factor_index_++;
      }
    }

  }

  // std::cout << "New factors size after landmarks: " << new_factors.size() << std::endl;

  // Check which calibration variables should be added
  auto ext_t_sigma = gtsam::noiseModel::Isotropic::Sigma(3, 
        noise_params_.extrinsic_translation_sigma_);

  for (const gtsam::Key& key : relevant_calib_keys) {
    if (!isam2_values.exists(key)) {
      auto ext_init = calib_values_init_.at<gtsam::Pose3>(key);
      new_values.insert(key, ext_init);

      auto ext_t_prior = boost::make_shared<gtsam::PriorFactorTranslation>(
        key, ext_init.translation(), ext_t_sigma);
      new_factors.add(ext_t_prior);
      curr_factor_index_++;
    }
  }

  // Add to ISAM2
  gtsam::ISAM2Result isam2_result_add = isam2_.update(
    new_factors, new_values, 
    gtsam::FactorIndices(), boost::none, calib_keys_);
}

 std::vector<gtsam::Pose3> IncrementalSegmentDatabase::getExtrinsicsEstimates() const {
   return SegmentDatabase::getExtrinsicsEstimates(isam2_.calculateBestEstimate());
 }

void IncrementalSegmentDatabase::retrieveEstimates(BackendOutput::shared_ptr& output_data) {
  gtsam::Values isam2_values = isam2_.calculateBestEstimate();
  SegmentDatabase::retrieveEstimates(isam2_values, output_data);

  // Marginals
  for (size_t l = 0; l < num_cam_; l++) {
    try {
      gtsam::Matrix6 I = isam2_.marginalInformation(TvcKey(l));
      output_data->marginal_information_.push_back(I);
    }
    catch (...) { // TODO: Specific types?
      // Negative 1s to indicate not in estimation?
      output_data->marginal_information_.push_back(-1.0*gtsam::Matrix6::Identity());
    }
  }
}

bool IncrementalSegmentDatabase::optimizeStep() {
  gtsam::ISAM2Result result = isam2_.update();

  bool converged = false;

  if (result.errorBefore) {
    // std::cout <<  *result.errorBefore << " " << *result.errorAfter << std::endl;
    // TODO: Setup user parameters?
    gtsam::LevenbergMarquardtParams params = gtsam::LevenbergMarquardtParams::CeresDefaults();
    if (gtsam::checkConvergence(params.relativeErrorTol, params.absoluteErrorTol, params.errorTol,
        *result.errorBefore, *result.errorAfter, params.verbosity) && std::isfinite(*result.errorAfter)) {
      converged = true;
    }
  }

  return converged;
}


void IncrementalSegmentDatabase::printPartitions() const {
  for (const auto& p : segment_partitions_) {
    for (const auto& seg_id : p) {
      std::cout << seg_id << " ";
    }
    std::cout << std::endl;
  }
}