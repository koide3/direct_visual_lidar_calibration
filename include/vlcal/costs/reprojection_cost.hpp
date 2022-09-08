#pragma once

#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <camera/generic_camera_base.hpp>

namespace vlcal {

class ReprojectionCost {
public:
  ReprojectionCost(const camera::GenericCameraBase::ConstPtr& proj, const Eigen::Vector3d& point_3d, const Eigen::Vector2d& point_2d)
  : proj(proj),
    point_3d(point_3d),
    point_2d(point_2d) {}

  ~ReprojectionCost() {}

  template <typename T>
  bool operator()(const T* const T_camera_lidar_params, T* residual) const {
    const Eigen::Map<Sophus::SE3<T> const> T_camera_lidar(T_camera_lidar_params);
    const Eigen::Matrix<T, 3, 1> pt_camera = T_camera_lidar * point_3d;

    const auto pt_2d = (*proj)(pt_camera);

    residual[0] = pt_2d[0] - point_2d[0];
    residual[1] = pt_2d[1] - point_2d[1];
    return true;
  }

private:
  const camera::GenericCameraBase::ConstPtr proj;
  const Eigen::Vector3d point_3d;
  const Eigen::Vector2d point_2d;
};

}  // namespace vlcal
