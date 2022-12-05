#pragma once

#include <camera/generic_camera_base.hpp>
#include <vlcal/common/frame_cpu.hpp>

namespace vlcal {

struct ViewCullingParams {
public:
  ViewCullingParams() {
    enable_depth_buffer_culling = true;
  }

  bool enable_depth_buffer_culling;       ///< If ture, perform depth-buffer-based hidden points removal
};

class ViewCulling {
public:
  ViewCulling(const camera::GenericCameraBase::ConstPtr& proj, const Eigen::Vector2i& image_size, const ViewCullingParams& params);
  ~ViewCulling();

  /**
   * @brief Remove points that should not be visible from the viewpoint of the camera
   */
  FrameCPU::Ptr cull(const Frame::ConstPtr& points, const Eigen::Isometry3d& T_camera_lidar) const;

private:
  std::vector<int> view_culling(const std::vector<int>& point_indices, const std::vector<Eigen::Vector4d>& points_camera) const;
  std::vector<int> hidden_points_removal(const std::vector<int>& point_indices, const std::vector<Eigen::Vector4d>& points_camera) const;

private:
  const ViewCullingParams params;

  const camera::GenericCameraBase::ConstPtr proj;
  const Eigen::Vector2i image_size;
  const double min_z;
};

}  // namespace vlcal
