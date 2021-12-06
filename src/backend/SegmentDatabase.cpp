#include "calibration/backend/SegmentDatabase.h"
#include <filesystem>
#include "calibration/utils/serialization.h"
#include "calibration/backend/partition.h"
#include "calibration/utils/gtsam_utils.h"
#include <gtsam/inference/LabeledSymbol.h>


SegmentDatabase::SegmentDatabase(
    const BackendParams::SegmentDatabaseParams& database_params,
    const BackendParams::NoiseModelParams& noise_params,
    const gtsam::TriangulationParameters& triangulation_params)
  : database_params_(database_params), 
    noise_params_(noise_params),
    triangulation_params_(triangulation_params),
    curr_sess_id_(0),
    seq_id_(0),
    info_heaps_(database_params.num_partitions_),
    can_optimize_(false)
{
}

SegmentDatabase::~SegmentDatabase() {
}

bool SegmentDatabase::loadDatabase(std::vector<gtsam::Pose3>& extrinsics) {
  // Load extrinsics if available, otherwise create noisy estimates for now
  // TODO: Parameter for loading gt vs noise?
  std::string extrinsics_path = database_params_.dir_ + std::string("extrinsics.dat");
  bool loaded_extrinsics = false;
  if (std::filesystem::exists(extrinsics_path)) {
    loaded_extrinsics = io::deserializeExtrinsics(extrinsics_path, extrinsics);
  }

  num_cam_ = extrinsics.size();

  for (int l = 0; l < extrinsics.size(); l++) {
    calib_values_init_.insert(TvcKey(l), extrinsics[l]);
    calib_keys_.push_back(TvcKey(l));
    std::cout << "Extrinsics init " << l << std::endl;
    std::cout << extrinsics[l].matrix() << std::endl;
  }
  this->addCalibrationVariables(extrinsics);
  
  std::cout << "LOADING DATABASE" << std::endl;

  // Get all filenames and then sort
  std::map<SegmentId, std::string> filenames;
  std::filesystem::path data_extension = ".dat";
  for (const auto& entry : std::filesystem::directory_iterator(database_params_.dir_)) {
    if (entry.path().extension().compare(data_extension) == 0
        && entry.path().filename() != std::filesystem::path("extrinsics.dat")) {
      
      std::string file_stem = entry.path().stem().string();
      size_t pos = file_stem.find_last_of("_");
      int sess_id = std::stoi(file_stem.substr(0, pos));
      int seq_id = std::stoi(file_stem.substr(pos+1)); 
      SegmentId seg_id(sess_id, seq_id);

      filenames.insert({seg_id, entry.path().string()});
    }
  }
  std::cout << "Loading database in order:" << std::endl;
  for (const auto& [seg_id, filename] : filenames) {
    std::cout << seg_id << " " << filename << std::endl;
  }

  int max_sess_id = -1;
  int prev_sess_id = -1;
  for (const auto& [seg_id, filename] : filenames) {
    std::cout << "Loading " << filename << std::endl;
    // Load segment
    Segment segment;
    io::deserializeSegment(filename, segment);

    // Clear session variables each time new session
    if (prev_sess_id != segment.getSessionId()) {
      rekey_map_.clear();
    }

    // TODO: Should we recalculate info content with current extrinsics estimate?
    // Put segment into database
    this->proposeSegment(segment);

    prev_sess_id = segment.getSessionId();
    max_sess_id = std::max(max_sess_id, static_cast<int>(segment.getSessionId()));
  }
  
  // Get current session id
  curr_sess_id_ = static_cast<size_t>(max_sess_id+1);

  if (curr_sess_id_ > 25) {
    throw std::runtime_error("SegmentDatabase::loadDatabase: \
        Session ID is too large.  Please clear the database.");
  }

  // Clear session variables one last time
  rekey_map_.clear();
  
  std::cout << "END LOADING DATABASE" << std::endl;

  return loaded_extrinsics;
}

bool SegmentDatabase::saveDatabase() const {
  std::cout << "SAVING SEGMENT DATABASE" << std::endl;

  // Remove existing files in folder
  std::filesystem::path data_extension = ".dat";
  for (const auto& entry : std::filesystem::directory_iterator(database_params_.dir_)) {
    if (entry.path().extension().compare(data_extension) == 0) {
      std::cout << "Removing " << entry.path() << std::endl;
      bool was_removed = std::filesystem::remove(entry.path());
    }
  }

  // Save segments
  for (const auto& [seg_id, segment] : segments_) {
    std::string filename = database_params_.dir_ 
      + std::to_string(segment.getSessionId()) + "_" 
      + std::to_string(segment.getSequenceId()) + ".dat";
    std::cout << "Saving " << filename << std::endl;
    io::serializeSegment(segment, filename);
  }

  return true;
}

bool SegmentDatabase::proposeSegment(Segment& new_segment) {

  // gttic_(SegmentDatabase_proposeSegment);

  // Check if segment is from current session (loaded database already has ID)
  bool was_loaded = new_segment.hasId();
  gtsam::FastList<gtsam::Key> no_relin_keys;
  if (!was_loaded) {
    new_segment.setId(curr_sess_id_, seq_id_);
  }
  else {
    // Do not relinearize calibration if was loaded from database
    // Want to maintain linearization point from when full optimization last happened
    no_relin_keys = calib_keys_;
    // TODO: This is not being used at all? Should it for incremental version?
  }

  if (!new_segment.hasInfo()) {
    throw std::runtime_error("SegmentDatabase::proposeSegment: \
        Segment should have calculated info");
  }

  std::cout << "Proposing segment " << new_segment.getId() << std::endl;

  // Note: Segment should have called calcInfoContent() already

  size_t num_partitions_added = 0;
  std::set<SegmentId> segment_ids_to_remove;
  // bool seg_added = proposeSegmentPerPartition(
  //     info_metrics, new_segment.getId(), num_partitions_added, segment_ids_to_remove);
  bool seg_added = proposeSegmentMinMaxEntropy(
      new_segment.getCalibInfo(), new_segment.getMargInfo(), new_segment.getId(), 
      num_partitions_added, segment_ids_to_remove);

  // Update segment variables and optimize
  if (seg_added) {
    // Add segment with number of partitions
    segment_counts_.emplace(new_segment.getId(), num_partitions_added);
    this->addSegment(new_segment);

    // Remove segments
    for (const auto& seg_id : segment_ids_to_remove) {
      segment_counts_.erase(seg_id);
    }
    this->removeSegments(segment_ids_to_remove);
  }

  // Check if can start optimizing yet
  int q_max_tmp;
  std::cout << this->getMaxDatabaseEntropy(q_max_tmp) << " " << database_params_.database_entropy_thresh_ << std::endl;
  if (!can_optimize_) {
    int q_max;
    if (this->getMaxDatabaseEntropy(q_max) < database_params_.database_entropy_thresh_) {
      can_optimize_ = true;
      std::cout << "Starting to optimize" << std::endl;
    }
  }

  // Increment seq_id_ if current session 
  // Otherwise loaded from database so don't increment sequence
  if (!was_loaded) {
    seq_id_++;
  }

  // std::cout << "Num unique segments: " << segments_.size() << std::endl; 
  return seg_added;
}

std::vector<gtsam::Pose3> SegmentDatabase::getExtrinsicsEstimates(const gtsam::Values& values) const {
  std::vector<gtsam::Pose3> extrinsics;
  for (size_t l = 0; l < num_cam_; l++) {
    if (values.exists(TvcKey(l))) {
      const gtsam::Pose3& ext = values.at<gtsam::Pose3>(TvcKey(l));
      extrinsics.push_back(ext);
    }
    else {
      extrinsics.push_back(calib_values_init_.at<gtsam::Pose3>(TvcKey(l)));
    }
  }
  return extrinsics;
}

void SegmentDatabase::retrieveEstimates(
    const gtsam::Values& values,
    BackendOutput::shared_ptr& output_data) {

  // std::cout << "SegmentDatabase::retrieveEstimates" << std::endl;

  // Fill in output data - only output current session data for visualization
  // Extrinsics
  output_data->extrinsics_ = this->getExtrinsicsEstimates();
  // Poses
  output_data->segment_poses_.resize(segments_.size());

  unsigned char curr_sess_char = getSessionChar(curr_sess_id_);
  int i = 0;
  for (const auto& [seg_id, segment] : segments_) {
    if (seg_id.sess_id_ == curr_sess_id_) {
      const gtsam::Values& pose_values = segment.getValues();
      for (const auto& [key, val] : pose_values) {
        gtsam::Symbol old_symbol(key);
        gtsam::LabeledSymbol new_symbol(old_symbol.chr(), curr_sess_char, old_symbol.index());
        output_data->segment_poses_[i].push_back(values.at<gtsam::Pose3>(new_symbol));
      }
    }
    i++;
  }

  // Landmarks
  gtsam::Values::ConstFiltered<gtsam::Point3> point_values 
      = values.filter<gtsam::Point3>();
  // TODO: Only send current session's point cloud
  for (const auto& [key, point] : point_values) {
    gtsam::LabeledSymbol sym(key);
    if (sym.label() == (getSessionChar(curr_sess_id_))) {
      if (sym.chr() == 'l')
        output_data->tracked_landmarks_.push_back(point);
      else if (sym.chr() == 's')
        output_data->stereo_landmarks_.push_back(point);
    }
  }

  // Segment information content
  for (const auto& [seg_id, segment] : segments_) {
    output_data->segment_info_content_.push_back(segment.getCalibInfo());
  }

  output_data->database_info_content_ = getConservativeDatabaseLogDet();

}

bool SegmentDatabase::proposeSegmentPerPartition(
    const std::vector<double>& new_info_metrics,
    const SegmentId& new_segment_id,
    size_t& num_partitions_added,
    std::set<SegmentId>& segment_ids_to_remove) {

  bool seg_added = false;
  // check if reached maximum capacity
  if (!this->atCapacity()) {
    // add segment as the one for all
    for (int q=0; q<database_params_.num_partitions_; q++) {
      info_heaps_[q].push({new_info_metrics[q], new_segment_id});
    }
    num_partitions_added = database_params_.num_partitions_;
    seg_added = true;
  }
  else {
    for (int q=0; q<database_params_.num_partitions_; q++) {
      const auto& max_elem = info_heaps_[q].top();
      // Check if new segment gives more information for this partition
      if (max_elem.first >= new_info_metrics[q]) {
        // Decrement ref counter and remove segment if needed
        const SegmentId& id = max_elem.second;
        size_t& ref_count = segment_counts_.at(id);
        ref_count--;
        if (ref_count == 0) {
          segment_ids_to_remove.insert(id);
        }
        // Increment counter
        seg_added = true;
        num_partitions_added++;

        info_heaps_[q].pop(); // This invalidates all references to this element
        info_heaps_[q].push({new_info_metrics[q], new_segment_id});
      }
    }
  }

  return seg_added;
}

bool SegmentDatabase::proposeSegmentMinMaxEntropy(
    const std::vector<double>& new_info_metrics,
    const std::vector<gtsam::Matrix3>& new_segment_marg_info,
    const SegmentId& new_segment_id,
    size_t& num_partitions_added,
    std::set<SegmentId>& segment_ids_to_remove) {

  bool seg_added = false;

  // If first segment, mark as max for all partitionss
  if (segments_.size() == 0) {
    for (int q=0; q<database_params_.num_partitions_; q++) {
      info_heaps_[q].push({new_info_metrics[q], new_segment_id});
    }
    num_partitions_added = 1;
    seg_added = true;
  }
  // If not at capacity, add segment and mark if it becomes max
  else if (!this->atCapacity()) {
    for (int q=0; q<database_params_.num_partitions_; q++) {
      const auto& max_elem = info_heaps_[q].top();
      if (new_info_metrics[q] > max_elem.first) {
        info_heaps_[q].pop();
        info_heaps_[q].push({new_info_metrics[q], new_segment_id});
      }
    }
    num_partitions_added = 1;
    seg_added = true;
  }
  // If at capacity, check if new segment can make least-certain partition more certain
  else {

    int q_max = -1;
    double max_uncertainty = this->getMaxDatabaseEntropy(q_max);

    // Get the segment that is limiting the most uncertain partition
    // Note: Copy instead of reference because invalidated later
    auto [max_d_opt, max_seg_id] = info_heaps_[q_max].top();

    // std::cout << "Partition with max entropy: " << q_max << std::endl;
    // std::cout << "Max d opt " << max_d_opt << " from segment " << max_seg_id << std::endl;
    // std::cout << "New segment info at q_max " << new_info_metrics[q_max] << std::endl;

    // Check if new segment improves on this partition
    if (max_d_opt > new_info_metrics[q_max]) {
      // std::cout << "New segment improves worst partition " << q_max << std::endl;
      // Also need to check that new segment does not make any partition worse off than the most uncertain one
      std::vector<gtsam::Matrix3> info_mats_new(database_params_.num_partitions_, gtsam::Matrix3::Zero());
      std::vector<double> max_d_opt_new(database_params_.num_partitions_, std::numeric_limits<double>::lowest() );
      std::vector<SegmentId> max_segment_ids_new(database_params_.num_partitions_);
      for (const auto& [seg_id, segment] : segments_) {
        if (seg_id != max_seg_id) {
          auto marg_info = segment.getMargInfo();
          auto seg_info_metrics = segment.getCalibInfo();
          for (int q=0; q<database_params_.num_partitions_; q++) {
            info_mats_new[q] += marg_info[q];
            if (seg_info_metrics[q] > max_d_opt_new[q]) {
              max_d_opt_new[q] = seg_info_metrics[q];
              max_segment_ids_new[q] = seg_id;
            }
          }
        }
      }
      for (int q=0; q<database_params_.num_partitions_; q++) {
        info_mats_new[q] += new_segment_marg_info[q];
        if (new_info_metrics[q] > max_d_opt_new[q]) {
            max_d_opt_new[q] = new_info_metrics[q];
            max_segment_ids_new[q] = new_segment_id;
        }
      }

      

      bool increases_max_entropy = false;
      for (int q = 0; q<database_params_.num_partitions_; q++) {
        // std::cout << "q " << q << " Previous uncertainty: " << log_dets[q] << " new uncertainty: " << log_dets_new[q] << std::endl;
        // std::cout << "Max uncertainity " << max_uncertainty << std::endl;
        double log_det_new = -std::log(info_mats_new[q].determinant());
        if (log_det_new > max_uncertainty) {
          increases_max_entropy = true;
          // break;
        }
      }

      // Add segment and remove other if does not make the max entropy worse
      if (!increases_max_entropy) {
        
        // std::cout << "New segment also does not make any partition even worse" << std::endl;
        // std::cout << "New segment " << new_segment_id << " old segment " << max_seg_id << std::endl;
        // Remove old segment from all and put new max in
        for (int q = 0; q<database_params_.num_partitions_; q++) {
          auto e = info_heaps_[q].top();
          // std::cout << "Before " << e.first << " " << e.second << std::endl;
          info_heaps_[q].pop();
          info_heaps_[q].push({max_d_opt_new[q], max_segment_ids_new[q]});
          // std::cout << "After " << max_d_opt_new[q] << " " << max_segment_ids_new[q] << std::endl;
        }

        num_partitions_added = 1;
        segment_ids_to_remove.insert(max_seg_id);
        seg_added = true;
      }
    }

  }

  return seg_added;
  
}

std::vector<double> SegmentDatabase::getConservativeDatabaseLogDet() {

  // Add up info matricces from each segment (assumes independence)
  std::vector<gtsam::Matrix3> info_mats(database_params_.num_partitions_, gtsam::Matrix3::Zero());
  for (const auto& [seg_id, segment] : segments_) {
    auto seg_info_metrics = segment.getMargInfo();
    for (int q=0; q<database_params_.num_partitions_; q++) {
      info_mats[q] += seg_info_metrics[q];
    }
  }
  std::vector<double> log_dets(database_params_.num_partitions_, 0);
  for (int q=0; q<database_params_.num_partitions_; q++) {
    log_dets[q] = -std::log(info_mats[q].determinant());
  }
  return log_dets;
}

double SegmentDatabase::getMaxDatabaseEntropy(int& q_max) {

  std::vector<double> log_dets = getConservativeDatabaseLogDet();

  // Find maximum (least-certain calibration partition)
  double max_uncertainty = std::numeric_limits<double>::lowest(); 
  for (int q=0; q<database_params_.num_partitions_; q++) {
    if (log_dets[q] > max_uncertainty) {
      max_uncertainty = log_dets[q];
      q_max = q;
    } 
  }

  return max_uncertainty;
}

bool SegmentDatabase::isEnoughLandmarkOverlap(
    const Segment& segment1, 
    const Segment& segment2) {

  // TODO: This check should really be for triangulated landmarks only
  size_t shared_landmarks = partition::countSharedLandmarks(
      segment1, segment2);
  return (shared_landmarks > database_params_.landmark_overlap_thresh_);
}

gtsam::NonlinearFactor::shared_ptr SegmentDatabase::createPriorFactor(
    size_t first_id, 
    unsigned char sess_char,
    const gtsam::Values& values) {

  auto prior_noise_model = gtsam::noiseModel::Isotropic::Sigma(6, 
        noise_params_.pose_prior_sigma_);
  
  gtsam::LabeledSymbol symbol(TwvKey(first_id, sess_char));

  auto prior_factor = boost::make_shared<gtsam::PriorFactor<gtsam::Pose3> >(
      gtsam::PriorFactor<gtsam::Pose3>(
          symbol,
          values.at<gtsam::Pose3>(symbol), 
          prior_noise_model) );

  return prior_factor;
}

unsigned char SegmentDatabase::getSessionChar(uint8_t sess_id) {
  return 'A' + static_cast<unsigned char>(sess_id);
}