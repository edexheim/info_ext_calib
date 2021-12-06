#ifndef SERIALIZATION_H
#define SERIALIZATION_H

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include "calibration/backend/Segment.h"
#include "calibration/utils/boost_serialization.h"

namespace io {

// TODO: Do we really need all of these?
// Templates?

bool serializeGraph(const gtsam::NonlinearFactorGraph& graph,
    const std::string& filename);

bool serializeValues(const gtsam::Values& values,
    const std::string& filename);

bool serializeSegment(const Segment& segment,
    const std::string& filename);

bool deserializeSegment(const std::string& filename,
    Segment& segment);

bool serializeExtrinsics(const std::vector<gtsam::Pose3>& extrinsics,
    const std::string& filename);

bool deserializeExtrinsics(const std::string& filename,
    std::vector<gtsam::Pose3>& extrinsics);
} // end namepsace io

#endif