// SPDX-License-Identifier: MIT
// Copyright (c) 2021  Kenji Koide (k.koide@aist.go.jp)

#pragma once

#include <memory>
#include <atomic>
#include <unordered_set>
#include <unordered_map>

#include <Eigen/Core>

#include <vlcal/common/frame.hpp>
#include <vlcal/common/frame_traits.hpp>
#include <vlcal/common/nearest_neighbor_search.hpp>
#include <vlcal/common/vector3i_hash.hpp>

namespace vlcal {

/**
 * @brief Container to hold points in a voxel
 */
struct LinearContainer {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<LinearContainer>;
  using ConstPtr = std::shared_ptr<const LinearContainer>;

  LinearContainer(const int lru_count);
  ~LinearContainer();

  int size() const { return points.size(); }
  void insert(const Eigen::Vector4d& point, const double insertion_dist_sq_thresh);
  void insert(const Frame& frame, const int i, const double insertion_dist_sq_thresh);

public:
  mutable std::atomic_int last_lru_count;
  int serial_id;

  std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> points;
  std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> normals;
  std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> covs;
  std::vector<double> intensities;
};

/**
 * @brief Voxel-based incremental nearest neighbor search
 *        Bai et al., "Faster-LIO: Lightweight Tightly Coupled Lidar-Inertial Odometry Using Parallel Sparse Incremental Voxels", IEEE RA-L, 2022
 * @note  Only the linear iVox is implemented
 * @note  To the compatibility with other nearest neighbor search methods, this implementation returns indices that encode the voxel and point IDs.
 *        The first ```point_id_bits``` (e.g., 32) bits of a point index represent the point ID, and the rest ```voxel_id_bits``` (e.g., 32) bits
 * represent the voxel ID that contains the point.
 */
struct iVox : public NearestNeighborSearch {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<iVox>;
  using ConstPtr = std::shared_ptr<const iVox>;

  /**
   * @brief Construct a new iVox
   *
   * @param voxel_resolution       Voxel resolution
   * @param insertion_dist_thresh  Minimum distance between points
   * @param lru_thresh             LRC caching threshold
   */
  iVox(const double voxel_resolution = 0.5, const double insertion_dist_thresh = 0.05, const int lru_thresh = 10);
  virtual ~iVox() override;

  /**
   * @brief Insert points and all available attributes into the iVox
   * @param frame   Input frame
   */
  virtual void insert(const Frame& frame);

  /**
   * @brief Find the closest point
   * @param pt          Query point
   * @param k_indices   Index output
   * @param k_sq_dists  Distance output
   * @return Number of found neighbors (0 or 1)
   */
  size_t nearest_neighbor_search(const double* pt, size_t* k_indices, double* k_sq_dists) const;

  /**
   * @brief Find k-nearest neighbors
   * @note  Indices encode voxel and point IDs of neighbor points
   *        First "voxel_id_bits" of an index indicates a sequential voxel ID
   *        Last "point_id_bits" of an index indicates a point ID in the voxel
   * @param pt          Query point
   * @param k           Number of neighbors
   * @param k_indices   Indices output
   * @param k_sq_dists  Distances output
   * @return Number of found neighbors
   */
  virtual size_t knn_search(const double* pt, size_t k, size_t* k_indices, double* k_sq_dists) const override;

  // Parameter setters
  void set_voxel_resolution(const double resolution) { voxel_resolution = resolution; }          ///< Voxel resolution
  void set_insertion_dist_thresh(const double dist) { insertion_dist_sq_thresh = dist * dist; }  ///< Minimum distance between points in a cell
  void set_lru_cycle(const int lru_cycle) { this->lru_cycle = lru_cycle; }                       ///< LRU clearing cycle
  void set_lru_thresh(const int lru_thresh) { this->lru_thresh = lru_thresh; }                   ///< LRU cache threshold (larger keeps more voxels)
  void set_neighbor_voxel_mode(const int mode) { offsets = neighbor_offsets(mode); }             ///< Neighboring voxel search mode (1/7/19/27)

  // Attribute check
  bool has_points() const { return points_available; }            ///< Check if the iVox has points
  bool has_normals() const { return normals_available; }          ///< Check if the iVox has normals
  bool has_covs() const { return covs_available; }                ///< Check if the iVox has covariances
  bool has_intensities() const { return intensities_available; }  ///< Check if the iVox has intensities

  // Point attribute accessors
  const Eigen::Vector4d& point(const size_t i) const;   ///< Get a point
  const Eigen::Vector4d& normal(const size_t i) const;  ///< Get the normal of a point
  const Eigen::Matrix4d& cov(const size_t i) const;     ///< Get the covariance of a point
  double intensity(const size_t i) const;               ///< Get the intensity of a point

  /// Number of voxels
  int num_voxels() const { return voxelmap.size(); }

  /// Extract all points in iVox
  virtual std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> voxel_points() const;
  virtual std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> voxel_normals() const;
  virtual std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> voxel_covs() const;

protected:
  inline size_t calc_index(const size_t voxel_id, const size_t point_id) const { return (voxel_id << point_id_bits) | point_id; }
  inline size_t voxel_id(const size_t i) const { return i >> point_id_bits; }                ///< Extract the point ID from an index
  inline size_t point_id(const size_t i) const { return i & ((1ul << point_id_bits) - 1); }  ///< Extract the voxel ID from an index

  const Eigen::Vector3i voxel_coord(const Eigen::Vector4d& point) const;
  std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>> neighbor_offsets(const int neighbor_voxel_mode) const;

protected:
  static constexpr int point_id_bits = 32;                  ///< Use the first 32 bits of point index to represent point ID
  static constexpr int voxel_id_bits = 64 - point_id_bits;  ///< Use the rest bits to represent voxel ID
  static_assert(sizeof(size_t) == 8, "size_t is not 8 bytes!! point_id_bits and voxel_id_bits should be tuned accordingly!!");

  bool points_available;
  bool normals_available;
  bool covs_available;
  bool intensities_available;

  double voxel_resolution;                                                          ///< Voxel resolution
  double insertion_dist_sq_thresh;                                                  ///< Minimum distance between points in a voxel
  int lru_cycle;                                                                    ///< LRU clearing check cycle
  int lru_thresh;                                                                   ///< LRU caching threshold
  std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>> offsets;  ///< Neighbor voxel offsets

  int lru_count;  ///< Counter to manage LRU voxel deletion

  using VoxelMap = std::unordered_map<
    Eigen::Vector3i,
    LinearContainer::Ptr,
    XORVector3iHash,
    std::equal_to<Eigen::Vector3i>,
    Eigen::aligned_allocator<std::pair<const Eigen::Vector3i, LinearContainer::Ptr>>>;

  VoxelMap voxelmap;                     ///< Voxelmap
  std::vector<LinearContainer*> voxels;  ///< Flattened voxelmap for linear indexing
};

namespace frame {

template <>
struct traits<iVox> {
  static bool has_points(const iVox& ivox) { return ivox.has_points(); }
  static bool has_normals(const iVox& ivox) { return ivox.has_normals(); }
  static bool has_covs(const iVox& ivox) { return ivox.has_covs(); }
  static bool has_intensities(const iVox& ivox) { return ivox.has_intensities(); }

  static const Eigen::Vector4d& point(const iVox& ivox, size_t i) { return ivox.point(i); }
  static const Eigen::Vector4d& normal(const iVox& ivox, size_t i) { return ivox.normal(i); }
  static const Eigen::Matrix4d& cov(const iVox& ivox, size_t i) { return ivox.cov(i); }
  static double intensity(const iVox& ivox, size_t i) { return ivox.intensity(i); }
};

}  // namespace frame

}  // namespace vlcal