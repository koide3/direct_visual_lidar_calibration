#pragma once

#include <opencv2/core.hpp>
#include <vlcal/common/frame.hpp>
#include <camera/generic_camera_base.hpp>

namespace vlcal {

/**
 * @brief Generate LiDAR intensity image and point index map
 * @param proj            Camera model
 * @param image_sie       Image size
 * @param T_camera_lidar  Camera-LiDAR transformation
 * @param points          LiDAR points
 * @return Pair of rendered intensity image (CV_64FC1) and point index map (CV_32SC1)
 */
std::pair<cv::Mat, cv::Mat> generate_lidar_image(
  const camera::GenericCameraBase::ConstPtr& proj,
  const Eigen::Vector2i& image_size,
  const Eigen::Isometry3d& T_camera_lidar,
  const Frame::ConstPtr& points);

}  // namespace vlcal
