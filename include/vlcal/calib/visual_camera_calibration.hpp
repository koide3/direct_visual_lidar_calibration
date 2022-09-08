#pragma once

#include <camera/generic_camera_base.hpp>
#include <vlcal/common/visual_lidar_data.hpp>

namespace vlcal {

struct VisualCameraCalibrationParams {
public:
  VisualCameraCalibrationParams() {
    max_outer_iterations = 10;
    max_inner_iterations = 256;

    delta_trans_thresh = 0.1;
    delta_rot_thresh = 0.5 * M_PI / 180.0;
  }

  int max_outer_iterations;
  int max_inner_iterations;

  double delta_trans_thresh;
  double delta_rot_thresh;

  std::function<void(const Eigen::Isometry3d& T_camera_lidar)> callback;
};

class VisualCameraCalibration {
public:
  VisualCameraCalibration(const camera::GenericCameraBase::ConstPtr& proj, const std::vector<VisualLiDARData::ConstPtr>& dataset, const VisualCameraCalibrationParams& params = VisualCameraCalibrationParams());

  Eigen::Isometry3d calibrate(const Eigen::Isometry3d& init_T_camera_lidar);

private:
  Eigen::Isometry3d estimate_pose(const Eigen::Isometry3d& init_T_camera_lidar);

private:
  const VisualCameraCalibrationParams params;

  const camera::GenericCameraBase::ConstPtr proj;
  const std::vector<VisualLiDARData::ConstPtr> dataset;
};

}  // namespace vlcal
