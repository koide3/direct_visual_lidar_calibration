#pragma once

#include <camera/generic_camera_base.hpp>
#include <gtsam_ext/types/frame_cpu.hpp>

namespace vlcal {

struct ViewCullingParams {
public:
  ViewCullingParams() {
    enable_depth_buffer_culling = true;

    remove_hidden_points = false;
    hidden_points_removal_max_z = 200.0;
  }

  bool enable_depth_buffer_culling;

  bool remove_hidden_points;
  double hidden_points_removal_max_z;
};

class ViewCulling {
public:
  ViewCulling(const camera::GenericCameraBase::ConstPtr& proj, const Eigen::Vector2i& image_size, const ViewCullingParams& params);
  ~ViewCulling();

  gtsam_ext::FrameCPU::Ptr cull(const gtsam_ext::Frame::ConstPtr& points, const Eigen::Isometry3d& T_camera_lidar) const;

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
