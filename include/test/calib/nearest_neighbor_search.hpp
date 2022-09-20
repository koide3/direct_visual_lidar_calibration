#pragma once

#include <Eigen/Core>
#include <opencv2/core.hpp>

// forward declaration
namespace nanoflann {

template <class T, class DataSource, typename _DistanceType>
class L2_Simple_Adaptor;

template <typename Distance, class DatasetAdaptor, int DIM, typename IndexType>
class KDTreeSingleIndexAdaptor;

}  // namespace nanoflann

namespace vlcal {

class NearestNeighborSearch {
public:
  NearestNeighborSearch() {}
  virtual ~NearestNeighborSearch() {}

  virtual size_t knn_search(const Eigen::Vector2d& pt, size_t k, Eigen::Vector2i* k_neighbors) const = 0;
};

class ImageSpaceNearestNeighborSearch : public NearestNeighborSearch {
public:
  ImageSpaceNearestNeighborSearch(const cv::Mat& image, const int max_radius);
  ~ImageSpaceNearestNeighborSearch() override;

  virtual size_t knn_search(const Eigen::Vector2d& pt, size_t k, Eigen::Vector2i* k_neighbors) const override;

private:
  cv::Mat build_x_blank(const cv::Mat& image) const;

private:
  const cv::Mat image;
  const cv::Mat x_blank;
  const int max_radius;
};

class KdTreeNearestNeighborSearch : public NearestNeighborSearch {
public:
  using Index = nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<double, KdTreeNearestNeighborSearch, double>, KdTreeNearestNeighborSearch, 2, size_t>;

  KdTreeNearestNeighborSearch(const cv::Mat& image);
  ~KdTreeNearestNeighborSearch() override;

  virtual size_t knn_search(const Eigen::Vector2d& pt, size_t k, Eigen::Vector2i* k_neighbors) const override;

  inline size_t kdtree_get_point_count() const { return points.size(); }
  inline double kdtree_get_pt(const size_t idx, const size_t dim) const { return points[idx][dim]; }
  template <class BBox>
  bool kdtree_get_bbox(BBox&) const {
    return false;
  }

private:
  std::vector<Eigen::Vector2d> points;
  std::unique_ptr<Index> index;
};
}  // namespace vlcal
