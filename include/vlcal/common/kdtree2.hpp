#pragma once

#include <memory>
#include <iostream>
#include <Eigen/Core>

#include <nanoflann.hpp>
#include <vlcal/common/frame_traits.hpp>
#include <vlcal/common/nearest_neighbor_search.hpp>

namespace vlcal {

/**
 * @brief KdTree-based nearest neighbor search
 * @note  This is nanoflann wrapper with traits-based point cloud interface.
 */
template <typename Frame>
struct KdTree2 : public NearestNeighborSearch {
public:
  using Index = nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<double, KdTree2<Frame>, double>, KdTree2<Frame>, 3, size_t>;

  KdTree2(const std::shared_ptr<const Frame>& frame)
  : frame(frame),
    search_eps(-1.0),
    index(new Index(3, *this, nanoflann::KDTreeSingleIndexAdaptorParams(10))) {
    if (frame::size(*frame) == 0) {
      std::cerr << "error: empty frame is given for KdTree2" << std::endl;
      std::cerr << "     : frame::size() may not be implemented" << std::endl;
    }

    index->buildIndex();
  }
  virtual ~KdTree2() override {}

  inline size_t kdtree_get_point_count() const { return frame::size(*frame); }
  inline double kdtree_get_pt(const size_t idx, const size_t dim) const { return frame::point(*frame, idx)[dim]; }

  template <class BBox>
  bool kdtree_get_bbox(BBox&) const {
    return false;
  }

  virtual size_t knn_search(const double* pt, size_t k, size_t* k_indices, double* k_sq_dists) const override {
    if (search_eps > 0.0) {
      nanoflann::KNNResultSet<double, size_t> result_set(k);
      result_set.init(k_indices, k_sq_dists);
      nanoflann::SearchParams search_params;
      search_params.eps = search_eps;
      index->findNeighbors(result_set, pt, search_params);
      return result_set.size();
    }

    return index->knnSearch(pt, k, k_indices, k_sq_dists);
  }

public:
  const std::shared_ptr<const Frame> frame;

  double search_eps;

  std::unique_ptr<Index> index;
};

}  // namespace vlcal
