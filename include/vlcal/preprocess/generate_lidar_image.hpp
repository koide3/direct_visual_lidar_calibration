#pragma once

#include <opencv2/core.hpp>
#include <camera/generic_camera.hpp>
#include <gtsam_ext/types/frame.hpp>

namespace vlcal {

std::pair<cv::Mat, cv::Mat> generate_lidar_image(
  const camera::GenericCameraBase::ConstPtr& proj,
  const Eigen::Vector2i& image_size,
  const Eigen::Isometry3d& T_camera_lidar,
  const gtsam_ext::Frame::ConstPtr& points);

}  // namespace vlcal
