#ifndef FEATUREDATABASE_H
#define FEATUREDATABASE_H

#include <unordered_map>
#include <Eigen/Dense>
#include <vector>
#include <list>
#include "calibration/frontend/Frame.h"
#include "calibration/utils/basic_types.h"

class FeatureDatabase {

  public:
    FeatureDatabase();
    ~FeatureDatabase();

    void addFeatures(const MultiFrameInput::shared_ptr& multi_frame);
    void removeFeatures(const std::list<size_t>& feature_ids);

    void addKeyframeId(size_t kf_id) {kf_ids_.insert(kf_id);}

    void getMatches(const Frame* frame,
                    std::vector<FeatureReference>& active_refs,
                    std::vector<cv::Point2f>& active_matches,
                    std::vector<FeatureReference>& inactive_refs,
                    std::vector<cv::Point2f>& inactive_matches) const;
    
    std::vector<FeatureReference> lookupIds(
        const std::vector<size_t> feature_ids) const;

  private:

    std::unordered_map<size_t, FeatureReference> feature_map_;

    std::set<size_t> kf_ids_;
};

#endif