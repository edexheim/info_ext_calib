#include "calibration/backend/partition.h"
#include <queue>

namespace partition {

// Segment mergeSegments(const Segment& segment1, const Segment& segment2) {
//   const gtsam::Values& values1 = segment1.getValues();
//   const gtsam::Values& values2 = segment2.getValues();
//   gtsam::Values combined_values;
//   combined_values.insert(values1);
//   combined_values.insert(values2);

// }

// Note: Assumes camera observations are independent
size_t countSharedLandmarks(const Segment& segment1, const Segment& segment2) {
  const CameraObservations obs1 = segment1.getObsGraph();
  const CameraObservations obs2 = segment2.getObsGraph();

  size_t num_shared_landmarks = countOverlap(obs1, obs2);
  return num_shared_landmarks;
}

Segment mergeSegments(const std::vector<const Segment*>& segments) {

  size_t num_segments = segments.size();

  // std::cout << "MERGING SEGMENTS: " << std::endl;
  // for (const auto& segment_ptr : segments)
    // std::cout << segment_ptr->getId() << std::endl;

  // Merge values, odometry and find first pose
  size_t first_id = std::numeric_limits<size_t>::max();
  gtsam::Values values;
  gtsam::NonlinearFactorGraph graph_odom;
  CameraObservations stereo_obs;
  FeatureIdMatches feature_id_matches;

  SegmentId prev_seg_id(std::numeric_limits<size_t>::max(), 0);
  for (const auto& segment_ptr : segments) {
    first_id = std::min(first_id, segment_ptr->getFirstId());
    values.insert(segment_ptr->getValues());
    // Only add first odom if consecutive
    gtsam::NonlinearFactorGraph odom_factors = segment_ptr->getOdomGraph();
    SegmentId seg_id = segment_ptr->getId();
    // Only add first odom if sequential, otherwise disconnected
    if (seg_id.isSequential(prev_seg_id)) 
      graph_odom.push_back(odom_factors[0]);
    graph_odom.push_back(odom_factors.begin()+1, odom_factors.end());
    const CameraObservations& segment_stereo_obs = segment_ptr->getStereoObs();
    stereo_obs.insert(segment_stereo_obs.begin(), segment_stereo_obs.end());

    const auto& id_matches = segment_ptr->getFeatureIdMatches();
    for (const auto& match : id_matches) {
      feature_id_matches.push_back(match);
    }

    prev_seg_id = seg_id;
  }

  // TODO TODO TODO TODO TODO TODO
  // Need to account for feature ids for shared landmarks

  // Merge observations
  CameraObservations merged_obs;

  // Queue stores pair of landmark id and segment index
  std::priority_queue<std::pair<size_t, size_t>, 
      std::vector<std::pair<size_t, size_t> >,
      std::greater<std::pair<size_t, size_t> >
      > min_heap;

  // Initialize iterators and min heap
  std::vector<CameraObservations::const_iterator> it_vec;
  std::vector<CameraObservations::const_iterator> it_end;
  for (int s = 0; s < num_segments; s++) {
    const CameraObservations& segment_obs = segments[s]->getObsGraph();
    it_vec.push_back( segment_obs.begin() );
    it_end.push_back( segment_obs.end() );
    if (it_vec[s] != it_end[s]) {
      min_heap.push({it_vec[s]->first, s});
    }
  }

  size_t num_merged = 0;
  // Merge observations
  while (!min_heap.empty()) {
    const auto [landmark_id, seg_ind] = min_heap.top();
    min_heap.pop();
    const ProjectionFactorBundle& factors = it_vec[seg_ind]->second;
    std::vector<size_t> used_seg_inds = {seg_ind};
    std::vector<ProjectionFactorBundle> other_factors; 
    while (!min_heap.empty() && (min_heap.top().first == landmark_id) ) {
      // Continue merging
      const auto [landmark_id_other, seg_ind_other] = min_heap.top();
      min_heap.pop();
      other_factors.push_back(it_vec[seg_ind_other]->second);
      used_seg_inds.push_back(seg_ind_other);
    }
    // Only create merged factor if more than one, otherwise use existing
    if (other_factors.size() == 0) {
      merged_obs.insert({landmark_id, factors});
    }
    else {
      ProjectionFactorBundle merged_factor = factors;
      for (const auto& vec : other_factors) {
        merged_factor.insert(merged_factor.end(), vec.begin(), vec.end());
      }
      merged_obs.insert({landmark_id, merged_factor});
    }
    // Push onto priority queue
    for (const size_t& s : used_seg_inds) {
      it_vec[s]++;
      if (it_vec[s] != it_end[s]) {
        min_heap.push({it_vec[s]->first, s});
      }
    }
  }
  
  Segment merged_segment(graph_odom, merged_obs, stereo_obs, feature_id_matches,
      values, first_id);
  // SegmentId is set so that we know sequence before counting landmarks
  // TODO: This is messy, should be improved
  merged_segment.setId(segments[0]->getSessionId(), segments[0]->getSequenceId());

  return merged_segment;
}

// TODO: Could be more efficient, no need to actually merge segments before optimization.
// Can order by segment id, and if switching away from smart factors, merging maybe not needed?
// Still need to organize factors so delayed triangulation can be performed though
std::vector<Segment> segmentsToPartitions(
    const std::map<SegmentId, Segment>& segments,
    size_t num_landmark_cut) {

  // Part 1: Group consecutive segments into partitions
  // TODO: Could this be more efficient, especially merging observations?
  std::vector<std::vector<const Segment*> > seg_groups;
  SegmentId prev_seg_id(std::numeric_limits<size_t>::max(), 0);
  for (const auto& [seg_id, segment] : segments) {
    if (seg_id.isSequential(prev_seg_id)) {
      // Add consecutive segment to most recent partition
      size_t group_ind = seg_groups.size() - 1;
      seg_groups[group_ind].push_back(&(segment));
    }
    else {
      // Create new partition
      seg_groups.push_back({&(segment)});
    }
    prev_seg_id = seg_id;
  }
  // Part 2: Merge consecutive segments into new segments
  // TODO: Maybe unneccessary, since we should just merge at the end
  //       For now, just gives simplicity
  std::vector<Segment> motion_segments;
  motion_segments.reserve(seg_groups.size());
  for (const auto& seg_group : seg_groups) {
    motion_segments.push_back( mergeSegments(seg_group) );
  }
  // std::cout << "Num motion segments: " << motion_segments.size() << std::endl;
  // for (int i=0; i<motion_segments.size(); i++) {
  //   std::cout << motion_segments[i].getId() << std::endl;
  //   const CameraObservations& segment_obs = motion_segments[i].getObsGraph();
  //   std::cout << " Num obs: " << segment_obs.size() << std::endl;
  // }

  // Part 3: Group segments into independent partitions  based on landmark covisibility
  // TODO: Improve efficiency, lots of segment copying!
  std::vector<Segment> P;
  for (const auto& S_k : motion_segments) {
    std::set<Segment> C = {S_k};
    std::vector<size_t> merge_ind;
    for (size_t i = 0; i < P.size(); i++) {
      const auto& segment = P[i];
      // Segments must be from the same sequence
      // TODO: Would have to be relaxed for loop closures
      if (segment.getSessionId() == S_k.getSessionId()) {
        // TODO: IF NOT ENOUGH SHARED, SHOULD STILL MERGE OBS?
        if (countSharedLandmarks(segment, S_k) > num_landmark_cut) {
          C.insert(segment);
          merge_ind.push_back(i);
        }
      }

    }

    // Remove original merged segment in reverse order to preserve indices
    for (int i = merge_ind.size()-1; i >=0; i--) {
      P.erase(P.begin() + merge_ind[i]);
    }
    // Only call merge if needed
    if (C.size() > 1) {
      std::vector<const Segment*> segment_ptrs;
      for (const auto& seg : C)
        segment_ptrs.push_back(&seg);
      Segment p_C = mergeSegments(segment_ptrs);
      P.push_back(p_C);
    }
    else {
      P.push_back(S_k);
    }
  }

  // std::cout << "Num partition segments: " << P.size() << std::endl;

  return P;
}

} // end namespace partition