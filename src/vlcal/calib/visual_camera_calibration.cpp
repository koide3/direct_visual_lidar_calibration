#include <vlcal/calib/visual_camera_calibration.hpp>

#include <boost/format.hpp>

#include <dfo/nelder_mead.hpp>
#include <gtsam/geometry/Pose3.h>

#include <vlcal/costs/nid_cost.hpp>
#include <vlcal/calib/view_culling.hpp>
#include <vlcal/calib/cost_calculator_nid.hpp>

#include <glk/pointcloud_buffer.hpp>
#include <guik/viewer/light_viewer.hpp>

namespace vlcal {

VisualCameraCalibration::VisualCameraCalibration(
  const camera::GenericCameraBase::ConstPtr& proj,
  const std::vector<VisualLiDARData::ConstPtr>& dataset,
  const VisualCameraCalibrationParams& params) : params(params), proj(proj), dataset(dataset) {}

Eigen::Isometry3d VisualCameraCalibration::calibrate(const Eigen::Isometry3d& init_T_camera_lidar) {
  Eigen::Isometry3d T_camera_lidar = init_T_camera_lidar;

  for (int i = 0; i < params.max_outer_iterations; i++) {
    const Eigen::Isometry3d new_T_camera_lidar = estimate_pose(T_camera_lidar);
    const Eigen::Isometry3d delta = new_T_camera_lidar.inverse() * T_camera_lidar;
    T_camera_lidar = new_T_camera_lidar;

    const double delta_t = delta.translation().norm();
    const double delta_r = Eigen::AngleAxisd(delta.linear()).angle();
    const bool converged = delta_t < params.delta_trans_thresh && delta_r < params.delta_rot_thresh;

    std::stringstream sst;
    sst << boost::format("delta_t: %.3f [m]  delta_r: %.3f [rad]") % delta_t % delta_r << std::endl;
    sst << (converged ? "Outer loop converged" : "Re-run inner optimization with the new viewpoint");
    guik::LightViewer::instance()->append_text(sst.str());

    if (converged) {
      break;
    }
  }

  return T_camera_lidar;
}

Eigen::Isometry3d VisualCameraCalibration::estimate_pose(const Eigen::Isometry3d& init_T_camera_lidar) {
  ViewCullingParams view_culling_params;
  view_culling_params.enable_depth_buffer_culling = !params.disable_z_buffer_culling;
  ViewCulling view_culling(proj, {dataset.front()->image.cols, dataset.front()->image.rows}, view_culling_params);

  std::vector<CostCalculator::Ptr> costs;
  for (const auto& data : dataset) {
    auto culled_points = view_culling.cull(data->points, init_T_camera_lidar);
    auto new_data = std::make_shared<VisualLiDARData>(data->image, culled_points);
    costs.emplace_back(std::make_shared<CostCalculatorNID>(proj, new_data));

    auto viewer = guik::LightViewer::instance();
    viewer->invoke([=] {
      auto culled_cloud_buffer = std::make_shared<glk::PointCloudBuffer>(culled_points->points, culled_points->size());
      culled_cloud_buffer->add_intensity(glk::COLORMAP::TURBO, culled_points->intensities, culled_points->size());
      auto raw_cloud_buffer = std::make_shared<glk::PointCloudBuffer>(data->points->points, data->points->size());
      auto sub = viewer->sub_viewer("view_culling");
      sub->set_draw_xy_grid(false);
      sub->set_camera_control(viewer->get_camera_control());
      sub->update_drawable("raw", raw_cloud_buffer, guik::FlatColor(0.2f, 0.2f, 0.2f, 1.0f));
      sub->update_drawable("culled", culled_cloud_buffer, guik::VertexColor().add("point_scale", 1.5f));
    });
  }

  double best_cost = std::numeric_limits<double>::max();

  const auto f = [&](const gtsam::Vector6& x) {
    const Eigen::Isometry3d T_camera_lidar = init_T_camera_lidar * Eigen::Isometry3d(gtsam::Pose3::Expmap(x).matrix());
    double sum_costs = 0.0;

#pragma omp parallel for reduction(+: sum_costs)
    for (int i = 0; i < costs.size(); i++) {
      sum_costs += costs[i]->calculate(T_camera_lidar);
    }

    if (sum_costs < best_cost) {
      best_cost = sum_costs;
      params.callback(T_camera_lidar);
      std::cout << "cost:" << best_cost << std::endl;
    }

    return sum_costs;
  };

  dfo::NelderMead<6>::Params nelder_mead_params;
  nelder_mead_params.init_step = params.nelder_mead_init_step;
  nelder_mead_params.convergence_var_thresh = params.nelder_mead_convergence_criteria;
  nelder_mead_params.max_iterations = params.max_inner_iterations;
  dfo::NelderMead<6> optimizer(nelder_mead_params);
  auto result = optimizer.optimize(f, gtsam::Vector6::Zero());

  const Eigen::Isometry3d T_camera_lidar = init_T_camera_lidar * Eigen::Isometry3d(gtsam::Pose3::Expmap(result.x).matrix());

  std::stringstream sst;
  sst << boost::format("Inner optimization terminated after %d iterations") % result.num_iterations << std::endl;
  sst << boost::format("Final cost: %.3f") % result.y << std::endl;
  sst << "--- T_camera_lidar ---" << std::endl << T_camera_lidar.matrix();

  guik::LightViewer::instance()->append_text(sst.str());

  return T_camera_lidar;
}

}  // namespace vlcal
