// SPDX-License-Identifier: MIT
// Copyright (c) 2021  Kenji Koide (k.koide@aist.go.jp)

#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <vlcal/common/frame.hpp>

namespace vlcal {

struct NearestNeighborSearch;

/**
 * @brief Continuous Time ICP Factor
 *        Bellenbach et al., "CT-ICP: Real-time Elastic LiDAR Odometry with Loop Closure", 2021
 */
template <typename TargetFrame = Frame, typename SourceFrame = Frame>
class IntegratedCT_ICPFactor_ : public gtsam::NonlinearFactor {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using shared_ptr = boost::shared_ptr<IntegratedCT_ICPFactor_<TargetFrame, SourceFrame>>;

  /**
   * @brief Constructor
   * @param source_t0_key   Key of the source pose at the beginning of the scan
   * @param source_t1_key   Key of the source pose at the end of the scan
   * @param target          Target point cloud
   * @param source          Source point cloud
   * @param target_tree     NN search for the target point cloud
   */
  IntegratedCT_ICPFactor_(
    gtsam::Key source_t0_key,
    gtsam::Key source_t1_key,
    const std::shared_ptr<const TargetFrame>& target,
    const std::shared_ptr<const SourceFrame>& source,
    const std::shared_ptr<NearestNeighborSearch>& target_tree);

  /**
   * @brief Constructor
   * @param source_t0_key   Key of the source pose at the beginning of the scan
   * @param source_t1_key   Key of the source pose at the end of the scan
   * @param target          Target point cloud
   * @param source          Source point cloud
   */
  IntegratedCT_ICPFactor_(
    gtsam::Key source_t0_key,
    gtsam::Key source_t1_key,
    const std::shared_ptr<const TargetFrame>& target,
    const std::shared_ptr<const SourceFrame>& source);

  virtual ~IntegratedCT_ICPFactor_() override;

  virtual size_t dim() const override { return 6; }
  virtual double error(const gtsam::Values& values) const override;
  virtual boost::shared_ptr<gtsam::GaussianFactor> linearize(const gtsam::Values& values) const override;

  void set_num_threads(int n) { num_threads = n; }
  void set_max_corresponding_distance(double dist) { max_correspondence_distance_sq = dist * dist; }

  const std::vector<double>& get_time_table() const { return time_table; }
  const std::vector<int>& get_time_indices() const { return time_indices; }
  const std::vector<gtsam::Pose3, Eigen::aligned_allocator<gtsam::Pose3>>& get_source_poses() const { return source_poses; }

  std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> deskewed_source_points(const gtsam::Values& values, bool local = false);

public:
  virtual void update_poses(const gtsam::Values& values) const;

protected:
  virtual void update_correspondences() const;

protected:
  int num_threads;
  double max_correspondence_distance_sq;

  std::shared_ptr<NearestNeighborSearch> target_tree;

  std::vector<double> time_table;
  mutable std::vector<gtsam::Pose3, Eigen::aligned_allocator<gtsam::Pose3>> source_poses;
  mutable std::vector<gtsam::Matrix6, Eigen::aligned_allocator<gtsam::Matrix6>> pose_derivatives_t0;
  mutable std::vector<gtsam::Matrix6, Eigen::aligned_allocator<gtsam::Matrix6>> pose_derivatives_t1;

  std::vector<int> time_indices;
  mutable std::vector<long> correspondences;

  std::shared_ptr<const TargetFrame> target;
  std::shared_ptr<const SourceFrame> source;
};

using IntegratedCT_ICPFactor = IntegratedCT_ICPFactor_<>;

}  // namespace vlcal

#include <vlcal/common/integrated_ct_icp_factor_impl.hpp>