#include "calibration/backend/graph_utils.h"
#include <unordered_map>
#include "calibration/utils/gtsam_utils.h"

namespace graph_utils {

static void dfs(
    int v, int cc_count,
    const std::vector<std::vector<size_t> >& adj,
    std::vector<int>& visited) {

  visited[v] = cc_count;

  for (const auto& it : adj[v]) {
    if (visited[it] == -1)
      dfs(it, cc_count, adj, visited);
  }
}

void getConnectedComponents(
    const std::vector<size_t>& ids,
    const FeatureIdMatches& feature_id_matches,
    std::vector<std::vector<size_t> >& connected_components) {

  size_t vertex_count = ids.size();

  // Construct graph 
  // Vertices
  std::unordered_map<size_t, size_t> vertex_map;
  for (size_t i = 0; i < ids.size(); i++) {
    vertex_map.insert({ids[i], i});
  }
  // Edges in adjacency list
  std::vector<std::vector<size_t> > adj(vertex_count);
  for (const auto& [id1, id2] : feature_id_matches) {
    size_t ind1 = vertex_map.at(id1);
    size_t ind2 = vertex_map.at(id2);
    adj[ind1].push_back(ind2);
    adj[ind2].push_back(ind1);
  }

  // Graph search via DFS
  std::vector<int> visited(vertex_count, -1);
  int cc_count = 0;
  for (int v = 0; v < vertex_count; v++) {
    if (visited[v] == -1) {
      dfs(v, cc_count, adj, visited);
      cc_count++;
    }
  }

  // if (vertex_map.find(1336) != vertex_map.end()) {
  //   size_t ind = vertex_map[1336];
  //   for (const auto& e : adj[ind])
  //     std::cout << ids[e] << std::endl;
  // }

  // Retrieve connected components
  connected_components.clear();
  connected_components.resize(cc_count);
  for (int i = 0; i < ids.size(); i++) {
    connected_components[visited[i]].push_back(ids[i]);
  }

}

void mergeObservations(
    const CameraObservations& obs_in,
    const std::vector<std::vector<size_t> >& connected_components,
    std::function<gtsam::Key(size_t id)> landmark_key_func,
    CameraObservations& obs_out) {
  
  for (const auto& vec : connected_components) {
    size_t first_id = vec.at(0);
    ProjectionFactorBundle factors;
    for (const auto& v : vec) {
      std::map<gtsam::Key, gtsam::Key> rekey_mapping;
      rekey_mapping.insert({landmark_key_func(v), landmark_key_func(first_id)});
      const auto& obs = obs_in.at(v);
      for (const auto& f : obs) {
        factors.push_back(
            boost::dynamic_pointer_cast<ProjectionFactor>(
                f->rekey(rekey_mapping) ) );
      }
    }
    obs_out.insert({first_id, factors});
  }
}

void mergeProjectionFactors(
    const CameraObservations& obs_in,
    const FeatureIdMatches& feature_id_matches,
    std::function<gtsam::Key(size_t id)> landmark_key_func,
    CameraObservations& merged_obs) {

  // Get connected feature_ids
  std::vector<size_t> feature_ids;
  for (const auto& p : obs_in)
    feature_ids.push_back(p.first);
  std::vector<std::vector<size_t> > connected_components;
  graph_utils::getConnectedComponents(feature_ids, feature_id_matches, 
      connected_components);

  // Merge projection factors
  graph_utils::mergeObservations(obs_in, connected_components, 
      landmark_key_func,
      merged_obs);
}

} // end namespace graph_utils