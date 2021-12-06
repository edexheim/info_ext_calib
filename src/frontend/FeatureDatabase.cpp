#include "calibration/frontend/FeatureDatabase.h"
#include <boost/graph/incremental_components.hpp>

// for debugging
#include <iostream>

FeatureDatabase::FeatureDatabase() {

}

FeatureDatabase::~FeatureDatabase() {

}

void FeatureDatabase::addFeatures(const MultiFrameInput::shared_ptr& multi_frame) {
  
  size_t frame_id = multi_frame->frame_id_;
  bool is_keyframe = (kf_ids_.find(frame_id) != kf_ids_.end());

  for (int l = 0; l < multi_frame->frames_.size(); l++) {
    const auto& frame = multi_frame->frames_[l];
    for (int i=0; i<frame->features_.size(); i++) {
      size_t ft_id = frame->feature_ids_[i];
      auto it = feature_map_.find(ft_id);
      // If feature is brand new, need to make a reference
      if (it == feature_map_.end()) {
        // std::cout << ft_id << std::endl;
        // TODO: Feature Id redundant as key and in struct
        feature_map_.insert({ft_id, FeatureReference(frame_id, frame->features_[i], ft_id)});
      }
      // Existing feature
      else if (is_keyframe) {
        // Overwrite if inactive reference
        bool prev_ref_inactive = (kf_ids_.find(it->second.frame_id_) == kf_ids_.end());
        if (prev_ref_inactive) {
          feature_map_[ft_id] = FeatureReference(frame_id, frame->features_[i], ft_id);
        }
      }
    }
  }
}

void FeatureDatabase::removeFeatures(const std::list<size_t>& feature_ids) {
  for (const auto& i : feature_ids) {
    feature_map_.erase(i);
  }
  // std::cout << "FeatureDatabase size: " << feature_map_.size() << std::endl;
}

void FeatureDatabase::getMatches( const Frame* frame,
                                  std::vector<FeatureReference>& active_refs,
                                  std::vector<cv::Point2f>& active_matches,
                                  std::vector<FeatureReference>& inactive_refs,
                                  std::vector<cv::Point2f>& inactive_matches) const {
  
  // note: refs and matches assumed to be empty, not cleared here
  for (int i=0; i<frame->features_.size(); i++) {
    size_t feature_id = frame->feature_ids_[i];
    const auto it = feature_map_.find(feature_id);
    if (it != feature_map_.end()) {
      if (kf_ids_.find(it->second.frame_id_) != kf_ids_.end()) {
        active_refs.push_back(it->second);
        active_matches.push_back(frame->features_[i]);
      }
      else {
        inactive_refs.push_back(it->second);
        inactive_matches.push_back(frame->features_[i]);
      }
    }
  }

}

std::vector<FeatureReference> FeatureDatabase::lookupIds(
    const std::vector<size_t> feature_ids) const {

  std::vector<FeatureReference> refs;
  refs.reserve(feature_ids.size());
  for (const size_t id : feature_ids) {
    refs.push_back(feature_map_.at(id));
  }
  return refs;
}