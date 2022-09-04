#include <vlcal/calib/nearest_neighbor_search.hpp>

#include <queue>
#include <nanoflann.hpp>

namespace vlcal {

ImageSpaceNearestNeighborSearch::ImageSpaceNearestNeighborSearch(const cv::Mat& image, const int max_radius) : image(image), max_radius(max_radius) {}

ImageSpaceNearestNeighborSearch::~ImageSpaceNearestNeighborSearch() {}

size_t ImageSpaceNearestNeighborSearch::knn_search(const Eigen::Vector2d& pt, size_t k, Eigen::Vector2i* k_neighbors) const {
  Eigen::Vector2i center = pt.cast<int>();

  const auto comp = [](const auto& lhs, const auto& rhs) { return lhs.first < rhs.first; };
  std::priority_queue<std::pair<double, Eigen::Vector2i>, std::vector<std::pair<double, Eigen::Vector2i>>, decltype(comp)> neighbors;

  for (int y_offset = -max_radius; y_offset <= max_radius; y_offset++) {
    const int y = center.y() + y_offset;
    if (y < 0 || y >= image.rows) {
      continue;
    }

    for (int x_offset = -max_radius; x_offset <= max_radius; x_offset++) {
      const int x = center.x() + x_offset;
      if (x < 0 || x >= image.cols) {
        continue;
      }

      const Eigen::Vector2i coord(x, y);
      if (!image.at<std::uint8_t>(coord.y(), coord.x())) {
        continue;
      }

      const double dist = (center - coord).squaredNorm();
      neighbors.emplace(dist, coord);
      if (neighbors.size() > k) {
        neighbors.pop();
      }
    }
  }

  const int num_neighbors = neighbors.size();
  for (int i = 0; i < num_neighbors; i++) {
    const auto [dist, coord] = neighbors.top();
    neighbors.pop();
    k_neighbors[num_neighbors - i - 1] = coord;
  }

  return num_neighbors;
}

KdTreeNearestNeighborSearch::KdTreeNearestNeighborSearch(const cv::Mat& image) {
  for (int i = 0; i < image.rows; i++) {
    for (int j = 0; j < image.cols; j++) {
      if (image.at<std::uint8_t>(i, j)) {
        points.emplace_back(j, i);
      }
    }
  }

  index.reset(new Index(2, *this, nanoflann::KDTreeSingleIndexAdaptorParams(10)));
  index->buildIndex();
}

KdTreeNearestNeighborSearch::~KdTreeNearestNeighborSearch() {}

size_t KdTreeNearestNeighborSearch::knn_search(const Eigen::Vector2d& pt, size_t k, Eigen::Vector2i* k_neighbors) const {
  std::vector<size_t> k_indices(k);
  std::vector<double> k_sq_dists(k);

  const int num_found = index->knnSearch(pt.data(), k, k_indices.data(), k_sq_dists.data());
  for (int i = 0; i < num_found; i++) {
    k_neighbors[i] = points[k_indices[i]].cast<int>();
  }

  return num_found;
}

}  // namespace vlcal
