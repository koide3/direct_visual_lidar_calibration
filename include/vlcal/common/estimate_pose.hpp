#pragma once

#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <camera/generic_camera_base.hpp>

namespace vlcal {

struct PoseEstimationParams {
  PoseEstimationParams() {
    ransac_iterations = 8192;
    ransac_error_thresh = 5.0;
  }

  int ransac_iterations;
  double ransac_error_thresh;
};

std::pair<int, Eigen::Matrix3d> estimate_rotation_ransac(
  const camera::GenericCameraBase::ConstPtr& proj,
  const std::vector<std::pair<Eigen::Vector2d, Eigen::Vector4d>>& correspondences,
  const double projection_error_thresh = 5.0,
  const int iterations = 8192);

class PoseEstimation {
public:
  PoseEstimation(const PoseEstimationParams& params = PoseEstimationParams());
  ~PoseEstimation();

  Eigen::Isometry3d
  estimate(const camera::GenericCameraBase::ConstPtr& proj, const std::vector<std::pair<Eigen::Vector2d, Eigen::Vector4d>>& correspondences, std::vector<bool>* inliers = nullptr);

private:
  Eigen::Matrix3d estimate_rotation_ransac(
    const camera::GenericCameraBase::ConstPtr& proj,
    const std::vector<std::pair<Eigen::Vector2d, Eigen::Vector4d>>& correspondences,
    std::vector<bool>* inliers);

  Eigen::Isometry3d estimate_pose_lsq(
    const camera::GenericCameraBase::ConstPtr& proj,
    const std::vector<std::pair<Eigen::Vector2d, Eigen::Vector4d>>& correspondences,
    const Eigen::Isometry3d& T_camera_lidar);

private:
  const PoseEstimationParams params;
};

}  // namespace vlcal
