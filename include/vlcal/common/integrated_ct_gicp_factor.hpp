#pragma once

#include <vlcal/common/integrated_ct_icp_factor.hpp>

namespace vlcal {

/**
 * @brief Continuous Time ICP with GICP's D2D cost
 *        Bellenbach et al., "CT-ICP: Real-time Elastic LiDAR Odometry with Loop Closure", 2021
 *        Segal et al., "Generalized-ICP", RSS2005
 */
template <typename TargetFrame = Frame, typename SourceFrame = Frame>
class IntegratedCT_GICPFactor_ : public IntegratedCT_ICPFactor_<TargetFrame, SourceFrame> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using shared_ptr = boost::shared_ptr<IntegratedCT_GICPFactor_<TargetFrame, SourceFrame>>;

  /**
   * @brief Constructor
   * @param source_t0_key   Key of the source pose at the beginning of the scan
   * @param source_t1_key   Key of the source pose at the end of the scan
   * @param target          Target point cloud
   * @param source          Source point cloud
   * @param target_tree     NN search for the target point cloud
   */
  IntegratedCT_GICPFactor_(
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
  IntegratedCT_GICPFactor_(
    gtsam::Key source_t0_key,
    gtsam::Key source_t1_key,
    const std::shared_ptr<const TargetFrame>& target,
    const std::shared_ptr<const SourceFrame>& source);

  virtual ~IntegratedCT_GICPFactor_() override;

  virtual double error(const gtsam::Values& values) const override;
  virtual boost::shared_ptr<gtsam::GaussianFactor> linearize(const gtsam::Values& values) const override;

protected:
  virtual void update_correspondences() const override;

  mutable std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> mahalanobis;
};

using IntegratedCT_GICPFactor = IntegratedCT_GICPFactor_<>;

}  // namespace vlcal

#include <vlcal/common/integrated_ct_gicp_factor_impl.hpp>
