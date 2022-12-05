#pragma once

#include <vector>

namespace vlcal {

/**
 * @brief Nearest neighbor search interface
 */
struct NearestNeighborSearch {
public:
  NearestNeighborSearch() {}
  virtual ~NearestNeighborSearch() {}

  /**
   * @brief k-nearest neighbor search
   * @param pt          Point
   * @param k           Number of neighbors
   * @param k_indices   Indices of k-nearest neighbors
   * @param k_sq_dists  Squared distances to the neighbors
   */
  virtual size_t knn_search(const double* pt, size_t k, size_t* k_indices, double* k_sq_dists) const { return 0; };
};
}  // namespace gtsam_ext