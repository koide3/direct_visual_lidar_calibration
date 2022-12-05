#pragma once

#include <vector>
#include <random>
#include <Eigen/Core>

#include <vlcal/common/frame.hpp>

namespace vlcal {

struct FrameCPU : public Frame {
public:
  using Ptr = std::shared_ptr<FrameCPU>;
  using ConstPtr = std::shared_ptr<const FrameCPU>;

  /**
   * @brief Constructor
   * @param points     Pointer to point data
   * @param num_points Number of points
   */
  template <typename T, int D>
  FrameCPU(const Eigen::Matrix<T, D, 1>* points, int num_points);

  /**
   * @brief Constructor
   * @param points  Points
   */
  template <typename T, int D, typename Alloc>
  FrameCPU(const std::vector<Eigen::Matrix<T, D, 1>, Alloc>& points) : FrameCPU(points.data(), points.size()) {}

  /// deep copy constructor
  FrameCPU(const Frame& frame);

  FrameCPU();
  ~FrameCPU();

  template <typename T>
  void add_times(const T* times, int num_points);
  template <typename T>
  void add_times(const std::vector<T>& times) {
    add_times(times.data(), times.size());
  }

  template <typename T, int D>
  void add_points(const Eigen::Matrix<T, D, 1>* points, int num_points);
  template <typename T, int D, typename Alloc>
  void add_points(const std::vector<Eigen::Matrix<T, D, 1>, Alloc>& points) {
    add_points(points.data(), points.size());
  }

  template <typename T, int D>
  void add_normals(const Eigen::Matrix<T, D, 1>* normals, int num_points);
  template <typename T, int D, typename Alloc>
  void add_normals(const std::vector<Eigen::Matrix<T, D, 1>, Alloc>& normals) {
    add_normals(normals.data(), normals.size());
  }

  template <typename T, int D>
  void add_covs(const Eigen::Matrix<T, D, D>* covs, int num_points);
  template <typename T, int D, typename Alloc>
  void add_covs(const std::vector<Eigen::Matrix<T, D, D>, Alloc>& covs) {
    add_covs(covs.data(), covs.size());
  }

  template <typename T>
  void add_intensities(const T* intensities, int num_points);
  template <typename T>
  void add_intensities(const std::vector<T>& intensities) {
    add_intensities(intensities.data(), intensities.size());
  }

  template <typename T>
  void add_aux_attribute(const std::string& attrib_name, const T* values, int num_points) {
    auto attributes = std::make_shared<std::vector<T>>(values, values + num_points);
    aux_attributes_storage[attrib_name] = attributes;
    aux_attributes[attrib_name] = std::make_pair(sizeof(T), attributes->data());
  }
  template <typename T, typename Alloc>
  void add_aux_attribute(const std::string& attrib_name, const std::vector<T, Alloc>& values) {
    add_aux_attribute(attrib_name, values.data(), values.size());
  }

  static FrameCPU::Ptr load(const std::string& path);

public:
  std::vector<double> times_storage;
  std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> points_storage;
  std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> normals_storage;
  std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> covs_storage;
  std::vector<double> intensities_storage;

  std::unordered_map<std::string, std::shared_ptr<void>> aux_attributes_storage;
};

/**
 * @brief Sample points
 * @param frame    Input points
 * @param indices  Point indices
 */
FrameCPU::Ptr sample(const Frame::ConstPtr& frame, const std::vector<int>& indices);

/**
 * @brief Naive random sampling
 * @param frame          Input points
 * @param sampling_rate  Random sampling rate in [0, 1]
 * @param mt             RNG
 * @return               Downsampled points
 */
FrameCPU::Ptr random_sampling(const Frame::ConstPtr& frame, const double sampling_rate, std::mt19937& mt);

/**
 * @brief Voxel grid downsampling
 * @note  This algorithm takes the average of point attributes (whatever it is) of each voxel
 *
 * @param frame             Input points
 * @param voxel_resolution  Voxel resolution
 */
FrameCPU::Ptr voxelgrid_sampling(const Frame::ConstPtr& frame, const double voxel_resolution);

/**
 * @brief Voxel grid random sampling
 * @note  This algorithm randomly samples points such that the number of sampled points of each voxel becomes (more or less) the same.
 *        This algorithm avoids mixing point attributes (unlike the standard voxelgrid downsampling), and thus can provide spatially
 *        well-distributed point samples with several attributes (e.g., normals and covs).
 *
 * @param frame             Input points
 * @param voxel_resolution  Voxel resolution
 * @param sampling_rate     Random sampling rate in [0, 1]
 * @param mt                RNG
 * @return                  Downsampled points
 */
FrameCPU::Ptr randomgrid_sampling(const Frame::ConstPtr& frame, const double voxel_resolution, const double sampling_rate, std::mt19937& mt);

/**
 * @brief Extract points for which pred returns true
 * @param frame  Input points
 * @param pred   Predicate function that takes Eigen::Vector4d and returns bool
 */
template <typename Func>
FrameCPU::Ptr filter(const Frame::ConstPtr& frame, const Func& pred) {
  std::vector<int> indices;
  indices.reserve(frame->size());
  for (int i = 0; i < frame->size(); i++) {
    if (pred(frame->points[i])) {
      indices.push_back(i);
    }
  }
  return sample(frame, indices);
}

/**
 * @brief Extract points for which pred returns true
 * @param frame  Input points
 * @param pred   Predicate function that takes a point index and returns bool
 */
template <typename Func>
FrameCPU::Ptr filter_by_index(const Frame::ConstPtr& frame, const Func& pred) {
  std::vector<int> indices;
  indices.reserve(frame->size());
  for (int i = 0; i < frame->size(); i++) {
    if (pred(i)) {
      indices.push_back(i);
    }
  }
  return sample(frame, indices);
}

/**
 * @brief Sort points
 * @param frame  Input points
 * @param pred   Comparison function that takes two point indices (lhs and rhs) and returns true if lhs < rhs
 */
template <typename Compare>
FrameCPU::Ptr sort(const Frame::ConstPtr& frame, const Compare& comp) {
  std::vector<int> indices(frame->size());
  std::iota(indices.begin(), indices.end(), 0);
  std::sort(indices.begin(), indices.end(), comp);
  return sample(frame, indices);
}

/**
 * @brief Sort points by time
 * @param frame  Input points
 */
FrameCPU::Ptr sort_by_time(const Frame::ConstPtr& frame);

/**
 * @brief Transform points, normals, and covariances
 *
 * @param frame            Input points
 * @param transformation   Transformation
 * @return Transformed points
 */
template <typename Scalar, int Mode>
FrameCPU::Ptr transform(const Frame::ConstPtr& frame, const Eigen::Transform<Scalar, 3, Mode>& transformation);

/**
 * @brief Transform points, normals, and covariances inplace
 * @param frame            [in/out] Points to be transformed
 * @param transformation   Transformation
 */
template <typename Scalar, int Mode>
void transform_inplace(Frame::Ptr& frame, const Eigen::Transform<Scalar, 3, Mode>& transformation);


}  // namespace vlcal
