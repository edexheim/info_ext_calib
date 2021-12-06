#pragma once

#include <map>
#include "calibration/backend/Segment.h"

namespace partition {

// Merging two maps of observations in O(n)
// Modified from https://stackoverflow.com/a/3773636
template<typename KeyType, typename Value>
size_t countOverlap(
    const std::map<KeyType, Value> & map1, 
    const std::map<KeyType, Value> & map2) {

  size_t num_intersection = 0;

  typename std::map<KeyType, Value>::const_iterator it1 = map1.begin();
  typename std::map<KeyType, Value>::const_iterator it2 = map2.begin();
  while (it1 != map1.end() && it2 != map2.end()) {
    if (it1->first < it2->first) {
      ++it1;
    }
    else if (it2->first < it1->first) {
      ++it2;
    }
    else {
      ++it1;
      ++it2;
      num_intersection++;
    }
  }
  return num_intersection;
}

size_t countSharedLandmarks(const Segment& segment1, const Segment& segment2);

Segment mergeSegments(const std::vector<const Segment*>& segments);

std::vector<Segment> segmentsToPartitions(
    const std::map<SegmentId, Segment>& segments,
    size_t num_landmark_cut);

} // end namespace partition