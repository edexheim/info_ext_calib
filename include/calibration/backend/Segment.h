#ifndef SEGMENT_H
#define SEGMENT_H

#include <unordered_map>
#include <unordered_set>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/VariableIndex.h>
#include "calibration/backend/SegmentId.h"
#include "calibration/camera_models/PinholeRadtan.h"

#include "calibration/utils/basic_types.h"
#include "calibration/backend/BackendParams.h"


class Segment {

  public:

    typedef std::shared_ptr<Segment> shared_ptr;

    /** default constructor - only used for serialization */
    Segment() {}

    Segment(const gtsam::NonlinearFactorGraph& graph_odom,
            const CameraObservations& graph_obs,
            const CameraObservations& stereo_obs,
            const FeatureIdMatches& feature_id_matches,
            const gtsam::Values& values,
            size_t first_id);

    ~Segment();

    // struct HashFunction {
    //   size_t operator()(const Segment& segment) const {
    //     return SegmentId::HashFunction(id_);
    //   }
    // };

    // Info content defaults to maximum, must be called explicitly
    void calcInfoContent(
        const std::vector<gtsam::Pose3>& extrinsics,
        const BackendParams::NoiseModelParams& noise_params,
        const gtsam::TriangulationParameters& triangulation_params,
        const BackendParams::MarginalCovarianceParams& marg_cov_params);

    // Accessors
    const gtsam::NonlinearFactorGraph& getOdomGraph() const {return graph_odom_;}
    const CameraObservations& getObsGraph() const {return graph_obs_;}
    const CameraObservations& getStereoObs() const {return stereo_obs_;}
    const FeatureIdMatches& getFeatureIdMatches() const {return feature_id_matches_;}
    const gtsam::Values& getValues() const {return values_;}
    size_t getFirstId() const {return first_id_;}
    bool hasId() const {return has_id_;}
    SegmentId getId() const {return id_;}
    size_t getSessionId() const {return id_.sess_id_;}
    size_t getSequenceId() const {return id_.seq_id_;}
    const std::vector<double>& getCalibInfo() const {return calib_info_;}
    const std::vector<gtsam::Matrix3>& getMargInfo() const {return marg_info_;}
    bool hasInfo() const {return has_info_;}

    // Mutators
    void setId(size_t sess_id, size_t seq_id) {
      id_ = SegmentId(sess_id, seq_id);
      has_id_ = true;
    }

    // Comparison operators (based on SegmentId)
    bool operator==(const Segment& other) const {return id_ == other.id_;}
  
    bool operator<(const Segment& other) const {return id_ < other.id_;}


  private:

    gtsam::Ordering getOrdering(const gtsam::VariableIndex& var_index, uint8_t num_cam) const;

    gtsam::NonlinearFactorGraph graph_odom_;
    CameraObservations graph_obs_;
    CameraObservations stereo_obs_;
    FeatureIdMatches feature_id_matches_;
    
    gtsam::Values values_;
    size_t first_id_;

    // Id and Info not required (such as after merging)
    bool has_id_;
    SegmentId id_;
    
    bool has_info_;
    std::vector<double> calib_info_;
    std::vector<gtsam::Matrix3> marg_info_;


    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
      ar & BOOST_SERIALIZATION_NVP(graph_odom_);
      ar & BOOST_SERIALIZATION_NVP(graph_obs_);
      ar & BOOST_SERIALIZATION_NVP(stereo_obs_);
      ar & BOOST_SERIALIZATION_NVP(feature_id_matches_);
      ar & BOOST_SERIALIZATION_NVP(values_);
      ar & BOOST_SERIALIZATION_NVP(first_id_);
      ar & BOOST_SERIALIZATION_NVP(has_id_);
      ar & BOOST_SERIALIZATION_NVP(id_);
      ar & BOOST_SERIALIZATION_NVP(has_info_);
      ar & BOOST_SERIALIZATION_NVP(calib_info_);
      ar & BOOST_SERIALIZATION_NVP(marg_info_);
  }
};

#endif