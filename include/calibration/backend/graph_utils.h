#pragma once

#include "calibration/utils/basic_types.h"

namespace graph_utils {

void getConnectedComponents(
    const std::vector<size_t>& ids,
    const FeatureIdMatches& feature_id_matches,
    std::vector<std::vector<size_t> >& connected_components);

void mergeObservations(
    const CameraObservations& obs_in,
    const std::vector<std::vector<size_t> >& connected_components,
    std::function<gtsam::Key(size_t id)> landmark_key_func,
    CameraObservations& obs_out);

void mergeProjectionFactors(
    const CameraObservations& obs_in,
    const FeatureIdMatches& feature_id_matches,
    std::function<gtsam::Key(size_t id)> landmark_key_func,
    CameraObservations& merged_obs);

} // end namespace graph_utils