#pragma once

#include <opencv2/core.hpp>
#include <gtsam_ext/types/frame_cpu.hpp>
#include <glk/pointcloud_buffer.hpp>
#include <camera/generic_camera.hpp>

namespace vlcal {

class PointsColorUpdater {
public:
  PointsColorUpdater(const camera::GenericCameraBase::ConstPtr& proj, const cv::Mat& image);

  PointsColorUpdater(const camera::GenericCameraBase::ConstPtr& proj, const cv::Mat& image, const gtsam_ext::FrameCPU::ConstPtr& points);

  void update(const Eigen::Isometry3d& T_camera_liar, const double blend_weight);

public:
  camera::GenericCameraBase::ConstPtr proj;
  double max_fov;

  cv::Mat image;

  gtsam_ext::FrameCPU::ConstPtr points;
  std::vector<Eigen::Vector4f> intensity_colors;
  std::shared_ptr<glk::PointCloudBuffer> cloud_buffer;
};

}  // namespace vlcal
