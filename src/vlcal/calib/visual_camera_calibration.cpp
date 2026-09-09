#include <vlcal/calib/visual_camera_calibration.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <boost/format.hpp>
#include <opencv2/opencv.hpp>

#include <ceres/ceres.h>
#include <ceres/autodiff_first_order_function.h>
#include <sophus/se3.hpp>
#include <sophus/ceres_manifold.hpp>
#include <dfo/nelder_mead.hpp>
#include <gtsam/geometry/Pose3.h>

#include <vlcal/costs/nid_cost.hpp>
#include <vlcal/calib/view_culling.hpp>
#include <vlcal/calib/cost_calculator_nid.hpp>

namespace vlcal {

VisualCameraCalibration::VisualCameraCalibration(
  const camera::GenericCameraBase::ConstPtr& proj,
  const std::vector<VisualLiDARData::ConstPtr>& dataset,
  const VisualCameraCalibrationParams& params)
: params(params), proj(proj), dataset(dataset) {}

void VisualCameraCalibration::log(const std::string& message) const {
  if (params.log_callback) {
    params.log_callback(message);
  } else {
    std::cout << message << std::endl;
  }
}

Eigen::Isometry3d VisualCameraCalibration::calibrate(const Eigen::Isometry3d& init_T_camera_lidar) {
  diagnostics_ = VisualCameraCalibrationDiagnostics();
  try {
    if (!proj || dataset.empty()) {
      throw std::runtime_error("calibration requires a camera and a nonempty dataset");
    }
    if (params.max_outer_iterations <= 0 || params.max_inner_iterations <= 0 || params.nid_bins < 2) {
      throw std::runtime_error("iteration limits must be positive and nid_bins must be at least 2");
    }
    if (!init_T_camera_lidar.matrix().allFinite()) {
      throw std::runtime_error("initial transform contains non-finite values");
    }
    for (const auto& data : dataset) {
      if (!data || data->image.empty() || data->image.type() != CV_8UC1 || !data->points || data->points->size() == 0 || !data->points->points || !data->points->intensities) {
        throw std::runtime_error("each calibration frame requires a grayscale image and nonempty points with intensities");
      }
    }

    Eigen::Isometry3d T_camera_lidar = init_T_camera_lidar;
    for (int i = 0; i < params.max_outer_iterations; i++) {
      diagnostics_.outer_iterations = i + 1;
      diagnostics_.visible_points.clear();
      Eigen::Isometry3d new_T_camera_lidar;
      switch (params.registration_type) {
        case RegistrationType::NID_BFGS:
          new_T_camera_lidar = estimate_pose_bfgs(T_camera_lidar);
          break;
        case RegistrationType::NID_NELDER_MEAD:
          new_T_camera_lidar = estimate_pose_nelder_mead(T_camera_lidar);
          break;
        default:
          throw std::runtime_error("unsupported registration type");
      }
      if (!new_T_camera_lidar.matrix().allFinite()) {
        throw std::runtime_error("optimizer returned a non-finite transform");
      }
      const Eigen::Isometry3d delta = new_T_camera_lidar.inverse() * T_camera_lidar;
      T_camera_lidar = new_T_camera_lidar;
      const double delta_t = delta.translation().norm();
      const double delta_r = Eigen::AngleAxisd(delta.linear()).angle();
      const bool converged = delta_t < params.delta_trans_thresh && delta_r < params.delta_rot_thresh;
      std::stringstream sst;
      sst << boost::format("delta_t: %.3f [m]  delta_r: %.3f [rad]") % delta_t % delta_r << std::endl;
      sst << (converged ? "Outer pose update below convergence threshold" : "Re-run inner optimization with the new viewpoint");
      log(sst.str());
      if (converged && diagnostics_.inner_converged) {
        diagnostics_.success = true;
        diagnostics_.converged = true;
        diagnostics_.termination_reason = "converged";
        return T_camera_lidar;
      }
    }
    throw std::runtime_error("maximum outer iterations reached without converged inner and outer optimization");
  } catch (const std::exception& error) {
    diagnostics_.success = false;
    diagnostics_.converged = false;
    diagnostics_.termination_reason = error.what();
    throw;
  }
}

Eigen::Isometry3d VisualCameraCalibration::estimate_pose_nelder_mead(const Eigen::Isometry3d& init_T_camera_lidar) {
  ViewCullingParams culling_params;
  culling_params.enable_depth_buffer_culling = !params.disable_z_buffer_culling;
  std::vector<CostCalculator::Ptr> costs;
  for (const auto& data : dataset) {
    ViewCulling culling(proj, {data->image.cols, data->image.rows}, culling_params);
    auto points = culling.cull(data->points, init_T_camera_lidar);
    diagnostics_.visible_points.push_back(points->size());
    if (points->size() == 0) {
      throw std::runtime_error("no image/LiDAR overlap after visibility culling");
    }
    auto culled_data = std::make_shared<VisualLiDARData>(data->image, points);
    NIDCostParams nid_params;
    nid_params.bins = params.nid_bins;
    costs.emplace_back(std::make_shared<CostCalculatorNID>(proj, culled_data, nid_params));
  }

  double best_cost = std::numeric_limits<double>::infinity();
  const auto f = [&](const gtsam::Vector6& x) {
    const Eigen::Isometry3d transform = init_T_camera_lidar * Eigen::Isometry3d(gtsam::Pose3::Expmap(x).matrix());
    double sum = 0.0;
#pragma omp parallel for reduction(+ : sum)
    for (int i = 0; i < static_cast<int>(costs.size()); i++) {
      sum += costs[i]->calculate(transform);
    }
    if (std::isfinite(sum) && sum < best_cost) {
      best_cost = sum;
      if (params.callback) params.callback(transform);
    }
    return std::isfinite(sum) ? sum : std::numeric_limits<double>::infinity();
  };
  const double initial_cost = f(gtsam::Vector6::Zero());
  if (!std::isfinite(initial_cost)) {
    throw std::runtime_error("NID is invalid at the initial pose (no overlap or insufficient intensity variation)");
  }
  if (diagnostics_.outer_iterations == 1) diagnostics_.initial_cost = initial_cost;

  dfo::NelderMead<6>::Params optimizer_params;
  optimizer_params.init_step = params.nelder_mead_init_step;
  optimizer_params.convergence_var_thresh = params.nelder_mead_convergence_criteria;
  optimizer_params.max_iterations = params.max_inner_iterations;
  dfo::NelderMead<6> optimizer(optimizer_params);
  const auto result = optimizer.optimize(f, gtsam::Vector6::Zero());
  diagnostics_.inner_iterations += result.num_iterations + 1;
  diagnostics_.inner_converged = result.converged;
  diagnostics_.final_cost = result.y;
  if (!std::isfinite(result.y) || !result.x.allFinite()) {
    throw std::runtime_error("Nelder-Mead returned a non-finite cost or pose");
  }
  const Eigen::Isometry3d transform = init_T_camera_lidar * Eigen::Isometry3d(gtsam::Pose3::Expmap(result.x).matrix());
  if (!std::isfinite(f(result.x))) {
    throw std::runtime_error("Nelder-Mead final pose has no valid NID overlap");
  }
  std::stringstream sst;
  sst << "Inner optimization (Nelder-Mead): " << (result.converged ? "converged" : "iteration limit") << std::endl;
  sst << "Final cost: " << result.y << std::endl << "--- T_camera_lidar ---" << std::endl << transform.matrix();
  log(sst.str());
  return transform;
}

struct MultiNIDCost {
  explicit MultiNIDCost(const Sophus::SE3d& initial) : initial(initial) {}
  void add(const std::shared_ptr<NIDCost>& cost) { costs.emplace_back(cost); }

  template <typename T>
  bool operator()(const T* params, T* residual) const {
    if (costs.empty()) return false;
    std::vector<double> values(Sophus::SE3d::num_parameters);
    std::transform(params, params + Sophus::SE3d::num_parameters, values.begin(), [](const auto& x) { return get_real(x); });
    if (!std::all_of(values.begin(), values.end(), [](double x) { return std::isfinite(x); })) return false;
    const Eigen::Map<const Sophus::SE3d> transform(values.data());
    const Sophus::SE3d delta = initial.inverse() * transform;
    if (delta.translation().norm() > 0.2 || Eigen::AngleAxisd(delta.rotationMatrix()).angle() > 2.0 * M_PI / 180.0) return false;

    // vector<bool> packs neighboring elements into shared words and is unsafe for parallel writes.
    std::vector<int> valid(costs.size());
    std::vector<T> residuals(costs.size(), T(0.0));
#pragma omp parallel for
    for (int i = 0; i < static_cast<int>(costs.size()); i++) valid[i] = (*costs[i])(params, &residuals[i]);
    if (std::count(valid.begin(), valid.end(), 0)) return false;
    *residual = T(0.0);
    for (const auto& value : residuals) *residual += value;
    return std::isfinite(get_real(*residual));
  }
  Sophus::SE3d initial;
  std::vector<std::shared_ptr<NIDCost>> costs;
};

struct IterationCallbackWrapper : public ceres::IterationCallback {
  explicit IterationCallbackWrapper(const std::function<ceres::CallbackReturnType(const ceres::IterationSummary&)>& callback) : callback(callback) {}
  ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary) override { return callback(summary); }
  std::function<ceres::CallbackReturnType(const ceres::IterationSummary&)> callback;
};

Eigen::Isometry3d VisualCameraCalibration::estimate_pose_bfgs(const Eigen::Isometry3d& init_T_camera_lidar) {
  ViewCullingParams culling_params;
  culling_params.enable_depth_buffer_culling = !params.disable_z_buffer_culling;
  Sophus::SE3d transform(init_T_camera_lidar.matrix());
  auto sum_nid = std::make_unique<MultiNIDCost>(transform);
  for (const auto& data : dataset) {
    ViewCulling culling(proj, {data->image.cols, data->image.rows}, culling_params);
    auto points = culling.cull(data->points, init_T_camera_lidar);
    diagnostics_.visible_points.push_back(points->size());
    if (points->size() == 0) throw std::runtime_error("no image/LiDAR overlap after visibility culling");
    cv::Mat normalized;
    data->image.convertTo(normalized, CV_64FC1, 1.0 / 255.0);
    sum_nid->add(std::make_shared<NIDCost>(proj, normalized, points, params.nid_bins));
  }
  double initial_cost;
  if (!(*sum_nid)(transform.data(), &initial_cost) || !std::isfinite(initial_cost)) {
    throw std::runtime_error("NID is invalid at the initial pose (no overlap or insufficient intensity variation)");
  }
  if (diagnostics_.outer_iterations == 1) diagnostics_.initial_cost = initial_cost;

  auto cost = new ceres::AutoDiffFirstOrderFunction<MultiNIDCost, Sophus::SE3d::num_parameters>(sum_nid.release());
  ceres::GradientProblem problem(cost, new Sophus::Manifold<Sophus::SE3>());
  ceres::GradientProblemSolver::Options options;
  options.minimizer_progress_to_stdout = true;
  options.update_state_every_iteration = true;
  options.line_search_direction_type = ceres::BFGS;
  options.max_num_iterations = params.max_inner_iterations;
  IterationCallbackWrapper callback([&](const ceres::IterationSummary&) {
    if (params.callback) params.callback(Eigen::Isometry3d(transform.matrix()));
    return ceres::SOLVER_CONTINUE;
  });
  if (params.callback) options.callbacks.push_back(&callback);

  ceres::GradientProblemSolver::Summary summary;
  ceres::Solve(options, problem, transform.data(), &summary);
  diagnostics_.inner_iterations += static_cast<int>(summary.iterations.size());
  diagnostics_.inner_converged = summary.termination_type == ceres::CONVERGENCE;
  diagnostics_.final_cost = summary.final_cost;
  log(summary.BriefReport());
  if (!summary.IsSolutionUsable() || !std::isfinite(summary.final_cost) || !transform.matrix().allFinite()) {
    throw std::runtime_error("Ceres BFGS failed: " + summary.BriefReport());
  }
  double verified_cost;
  if (!cost->Evaluate(transform.data(), &verified_cost, nullptr) || !std::isfinite(verified_cost)) {
    throw std::runtime_error("Ceres final pose has no valid NID overlap");
  }
  diagnostics_.final_cost = verified_cost;
  std::stringstream sst;
  sst << "Final cost: " << verified_cost << std::endl << "--- T_camera_lidar ---" << std::endl << transform.matrix();
  log(sst.str());
  return Eigen::Isometry3d(transform.matrix());
}

}  // namespace vlcal
