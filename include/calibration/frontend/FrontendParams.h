#ifndef FRONTEND_PARAMS_H
#define FRONTEND_PARAMS_H

#include <string>
#include <boost/serialization/nvp.hpp>
#include <opencv2/core/types.hpp>

class FrontendParams {

  public:

    FrontendParams(const std::string& path);
    ~FrontendParams();
    void loadYaml(const std::string& path);

    struct DetectorParams {
      cv::Size grid_size_;
      int num_features_per_cell_;
      int min_dist_;
    } detector_;

    enum TrackerType {KLT, ORB, KLT_ORB} tracker_type_;

    struct TrackerParams {
      // SparsePyrLKOpticalFlow only
      int window_size_;
      int max_level_;
      int max_iter_;
      double convergence_eps_;
      double min_eig_thresh_;
      bool two_way_tracking_;
      double two_way_thresh_;
      int pattern_;
    } tracker_;

    struct OutRejParams {
      double ransac_thresh_;
      double ransac_prob_;
      enum Model {SevenPoint, FivePoint} model_;
    } out_rej_;

    struct KeyframeSelecterParams {
      enum KfType {Entropy, Feature, Motion} keyframe_type_;
      double keyframe_prob_;
    } kf_seleceter_;

    struct UndistortParams {
      // GTSAM defaults
      UndistortParams()
        : max_iter_(10), tol_(1e-5), throw_err_(true)
      {}

      /** Serialization function */
      friend class boost::serialization::access;
      template<class Archive>
      void serialize(Archive & ar, const unsigned int /*version*/)
      {
        ar & BOOST_SERIALIZATION_NVP(max_iter_);
        ar & BOOST_SERIALIZATION_NVP(tol_);
        ar & BOOST_SERIALIZATION_NVP(throw_err_);
      }
      
      int max_iter_;
      double tol_;
      bool throw_err_;
    } undistort_;

    // Accessors
    const DetectorParams& getDetectorParams() const {return detector_;}
    const TrackerParams& getTrackerParams() const {return tracker_;}
    const OutRejParams& getOutRejParams() const {return out_rej_;}
    const KeyframeSelecterParams& getKeyframeSelecterParams() const {return kf_seleceter_;}
    const UndistortParams& getUndistortParams() const {return undistort_;}
};

#endif