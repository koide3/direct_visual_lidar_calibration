#pragma once

#include <vector>
#include <Eigen/Core>
#include <camera/generic_camera.hpp>

namespace vlcal {

std::pair<int, Eigen::Matrix3d> estimate_rotation_ransac(
  const camera::GenericCameraBase::ConstPtr& proj,
  const std::vector<std::pair<Eigen::Vector2d, Eigen::Vector4d>>& correspondences,
  const double projection_error_thresh = 5.0,
  const int iterations = 8192);

}  // namespace vlcal
