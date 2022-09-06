#pragma once

#include <camera/generic_camera.hpp>
#include <gtsam_ext/types/frame.hpp>

namespace vlcal {

double estimate_camera_fov(const camera::GenericCameraBase::ConstPtr& proj, const Eigen::Vector2i& image_size);

double estimate_lidar_fov(const gtsam_ext::Frame::ConstPtr& points);

}  // namespace vlcal
