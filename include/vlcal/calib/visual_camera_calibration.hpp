#pragma once

#include <camera/generic_camera_base.hpp>
#include <vlcal/common/visual_lidar_data.hpp>

namespace vlcal {

enum class RegistrationType { NID_BFGS, NID_NELDER_MEAD };

struct VisualCameraCalibrationParams {
public:
  VisualCameraCalibrationParams() {
    max_outer_iterations = 10;
    max_inner_iterations = 256;

    delta_trans_thresh = 0.1;
    delta_rot_thresh = 0.5 * M_PI / 180.0;

    disable_z_buffer_culling = false;

    nid_bins = 16;

    registration_type = RegistrationType::NID_BFGS;
    nelder_mead_init_step = 1e-3;
    nelder_mead_convergence_criteria = 1e-8;
  }

  int max_outer_iterations;   ///< Maximum number of outer iterations
  int max_inner_iterations;   ///< Maximum number of inner iterations (Nelder-Mead iterations)
  double delta_trans_thresh;  ///< Convergence criteria of the outer loop
  double delta_rot_thresh;    ///<

  bool disable_z_buffer_culling;  ///< If true, do not remove hidden points
  int nid_bins;                   ///< Number of histogram bins for NID computation

  RegistrationType registration_type;
  double nelder_mead_init_step;
  double nelder_mead_convergence_criteria;

  std::function<void(const Eigen::Isometry3d& T_camera_lidar)> callback;
};

class VisualCameraCalibration {
public:
  VisualCameraCalibration(const camera::GenericCameraBase::ConstPtr& proj, const std::vector<VisualLiDARData::ConstPtr>& dataset, const VisualCameraCalibrationParams& params = VisualCameraCalibrationParams());

  Eigen::Isometry3d calibrate(const Eigen::Isometry3d& init_T_camera_lidar);

private:
  Eigen::Isometry3d estimate_pose_nelder_mead(const Eigen::Isometry3d& init_T_camera_lidar);
  Eigen::Isometry3d estimate_pose_bfgs(const Eigen::Isometry3d& init_T_camera_lidar);

private:
  const VisualCameraCalibrationParams params;

  const camera::GenericCameraBase::ConstPtr proj;
  const std::vector<VisualLiDARData::ConstPtr> dataset;
};

}  // namespace vlcal
