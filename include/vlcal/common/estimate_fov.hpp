#pragma once

#include <camera/generic_camera.hpp>
#include <gtsam_ext/types/frame.hpp>

namespace vlcal {

Eigen::Vector3d estimate_direction(const camera::GenericCameraBase::ConstPtr& proj, const Eigen::Vector2d& pt_2d);

double estimate_camera_fov(const camera::GenericCameraBase::ConstPtr& proj, const Eigen::Vector2i& image_size);

double estimate_lidar_fov(const gtsam_ext::Frame::ConstPtr& points);

}  // namespace vlcal
