#pragma once

#include <opencv2/core.hpp>
#include <vlcal/common/frame_cpu.hpp>
#include <glk/pointcloud_buffer.hpp>
#include <camera/generic_camera_base.hpp>

namespace vlcal {

/**
 * @brief A class to colorize LiDAR points with a camera image
 */
class PointsColorUpdater {
public:
  PointsColorUpdater(const camera::GenericCameraBase::ConstPtr& proj, const cv::Mat& image);

  PointsColorUpdater(const camera::GenericCameraBase::ConstPtr& proj, const cv::Mat& image, const FrameCPU::ConstPtr& points);

  void update(const Eigen::Isometry3d& T_camera_liar, const double blend_weight);

public:
  camera::GenericCameraBase::ConstPtr proj;
  double min_nz;

  cv::Mat image;

  FrameCPU::ConstPtr points;
  std::vector<Eigen::Vector4f> intensity_colors;
  std::shared_ptr<glk::PointCloudBuffer> cloud_buffer;
};

}  // namespace vlcal
