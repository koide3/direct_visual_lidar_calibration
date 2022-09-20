#include <vlcal/calib/nearest_neighbor_search.hpp>

#include <queue>
#include <nanoflann.hpp>

#include <opencv2/opencv.hpp>

namespace vlcal {

ImageSpaceNearestNeighborSearch::ImageSpaceNearestNeighborSearch(const cv::Mat& image, const int max_radius)
: image(image),
  x_blank(build_x_blank(image)),
  max_radius(max_radius) {}

ImageSpaceNearestNeighborSearch::~ImageSpaceNearestNeighborSearch() {}

cv::Mat ImageSpaceNearestNeighborSearch::build_x_blank(const cv::Mat& image) const {
  cv::Mat blank(image.rows, image.cols, CV_32SC1, cv::Scalar::all(0));

  int max_count = 0;

  for (int y = 0; y < image.rows; y++) {
    int count = 0;
    for (int x_ = 0; x_ < image.cols; x_++) {
      const int x = image.cols - x_ - 1;
      if (image.at<std::uint8_t>(y, x)) {
        count = 0;
        blank.at<std::int32_t>(y, x) = 0;
      } else {
        blank.at<std::int32_t>(y, x) = count++;
      }

      max_count = std::max(max_count, count);
    }
  }

  return blank;
}

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
        const int step = x_blank.at<std::int32_t>(y, x);
        x_offset += step;
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
