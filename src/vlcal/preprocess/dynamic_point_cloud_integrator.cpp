#include <vlcal/preprocess/dynamic_point_cloud_integrator.hpp>

#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <vlcal/common/ivox.hpp>
#include <vlcal/common/kdtree2.hpp>
#include <vlcal/common/frame_cpu.hpp>
#include <vlcal/common/integrated_ct_gicp_factor.hpp>
#include <vlcal/common/vector3i_hash.hpp>

#include <vlcal/common/cloud_covariance_estimation.hpp>

#include <glk/pointcloud_buffer.hpp>
#include <glk/primitives/primitives.hpp>
#include <guik/viewer/light_viewer.hpp>

namespace vlcal {

DynamicPointCloudIntegratorParams::DynamicPointCloudIntegratorParams() {
  visualize = false;
  verbose = false;
  num_threads = 16;
  k_neighbors = 20;
  target_num_points = 10000;
  voxel_resolution = 0.05;
  min_distance = 1.0;
}

DynamicPointCloudIntegratorParams::~DynamicPointCloudIntegratorParams() {}

DynamicPointCloudIntegrator::DynamicPointCloudIntegrator(const DynamicPointCloudIntegratorParams& params) : params(params) {
  last_T_odom_lidar_begin = gtsam::Pose3();
  last_T_odom_lidar_end = gtsam::Pose3();

  target_ivox.reset(new iVox(1.0, 0.05, 100));

  voxelgrid_thread = std::thread([this] { voxelgrid_task(); });
}

DynamicPointCloudIntegrator::~DynamicPointCloudIntegrator() {
  alignment_results.submit_end_of_data();
  if (voxelgrid_thread.joinable()) {
    voxelgrid_thread.join();
  }
}

void DynamicPointCloudIntegrator::insert_points(const Frame::ConstPtr& raw_points_) {
  auto raw_points = sort_by_time(raw_points_);
  auto points = randomgrid_sampling(raw_points, 0.5, static_cast<double>(params.target_num_points) / raw_points->size(), mt);

  std::vector<int> neighbors(points->size() * params.k_neighbors);

  // Find kNN
  KdTree2<Frame> tree(points);
#pragma omp parallel for num_threads(params.num_threads)
  for (int i = 0; i < points->size(); i++) {
    std::vector<size_t> k_indices(params.k_neighbors);
    std::vector<double> k_sq_dists(params.k_neighbors);
    tree.knn_search(points->points[i].data(), params.k_neighbors, k_indices.data(), k_sq_dists.data());
    std::copy(k_indices.begin(), k_indices.end(), neighbors.begin() + params.k_neighbors * i);
  }

  // Estimate covariances
  CloudCovarianceEstimation covariance_estimation(params.num_threads);
  points->add_covs(covariance_estimation.estimate(points->points_storage, neighbors));

  if (!target_ivox->has_points()) {
    // Handling the first frame
    target_ivox->insert(*points);
    alignment_results.push(std::make_tuple(raw_points, gtsam::Pose3(), gtsam::Pose3()));
    return;
  }

  const double scan_duration = raw_points->times[raw_points->size() - 1];
  const gtsam::Vector6 pred_v_odom_lidar = gtsam::Pose3::Logmap(last_T_odom_lidar_begin.between(last_T_odom_lidar_end)) / scan_duration;
  const gtsam::Pose3 pred_T_odom_lidar_begin = last_T_odom_lidar_end;
  const gtsam::Pose3 pred_T_odom_lidar_end = last_T_odom_lidar_end * gtsam::Pose3::Expmap(0.75 * pred_v_odom_lidar * scan_duration);

  gtsam::Values values;
  values.insert(0, pred_T_odom_lidar_begin);
  values.insert(1, pred_T_odom_lidar_end);

  gtsam::NonlinearFactorGraph graph;
  auto factor = boost::make_shared<IntegratedCT_GICPFactor_<iVox, Frame>>(0, 1, target_ivox, points, target_ivox);
  factor->set_num_threads(params.num_threads);
  graph.add(factor);

  graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(0, gtsam::Pose3(pred_T_odom_lidar_begin), gtsam::noiseModel::Isotropic::Precision(6, 1e3));
  graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(0, 1, gtsam::Pose3(), gtsam::noiseModel::Isotropic::Precision(6, 1e5));

  gtsam::LevenbergMarquardtParams lm_params;
  lm_params.setMaxIterations(10);
  if (params.verbose) {
    lm_params.setVerbosityLM("SUMMARY");
  }

  for (int i = 0; i < 3; i++) {
    values = gtsam::LevenbergMarquardtOptimizer(graph, values, lm_params).optimize();
  }

  last_T_odom_lidar_begin = values.at<gtsam::Pose3>(0);
  last_T_odom_lidar_end = values.at<gtsam::Pose3>(1);

  auto deskewed = std::make_shared<FrameCPU>(factor->deskewed_source_points(values));
  deskewed->add_covs(covariance_estimation.estimate(deskewed->points_storage, neighbors));
  deskewed->add_intensities(points->intensities, points->size());
  target_ivox->insert(*deskewed);

  alignment_results.push(std::make_tuple(raw_points, values.at<gtsam::Pose3>(0), values.at<gtsam::Pose3>(1)));

  if (params.visualize) {
    auto viewer = guik::LightViewer::instance();
    viewer->update_drawable("coord", glk::Primitives::coordinate_system(), guik::VertexColor(last_T_odom_lidar_end.matrix()));
    viewer->update_drawable("points", std::make_shared<glk::PointCloudBuffer>(deskewed->points, deskewed->size()), guik::FlatOrange());
    viewer->update_drawable("target", std::make_shared<glk::PointCloudBuffer>(target_ivox->voxel_points()), guik::Rainbow());
    viewer->spin_once();
  }
}

void DynamicPointCloudIntegrator::voxelgrid_task() {
  while (true) {
    auto data = alignment_results.pop_lock();
    if (!data) {
      break;
    }

    const auto& raw_points = std::get<0>(*data);
    const auto& T_odom_lidar_begin = std::get<1>(*data);
    const auto& T_odom_lidar_end = std::get<2>(*data);
    const double max_timestamp = raw_points->times[raw_points->size() - 1];

    double last_t = -1.0;
    gtsam::Pose3 T_odom_lidar = T_odom_lidar_begin;

    const double time_eps = 1e-4;
    for (int i = 0; i < raw_points->size(); i++) {
      const double t = raw_points->times[i] / max_timestamp;

      if (t - last_t > time_eps) {
        last_t = t;
        T_odom_lidar = T_odom_lidar_begin.interpolateRt(T_odom_lidar_end, t);
      }

      const Eigen::Vector4d pt = T_odom_lidar.matrix() * raw_points->points[i];

      if (pt.head<3>().norm() < params.min_distance) {
        continue;
      }

      const Eigen::Vector3i coord = (pt / params.voxel_resolution).array().floor().cast<int>().head<3>();
      voxelgrid[coord] = Eigen::Vector4d(pt[0], pt[1], pt[2], raw_points->intensities[i]);
    }

    /*
    std::vector<Eigen::Vector3f> points;
    std::vector<float> intensities;

    voxelgrid->forEachCell([&](const Eigen::Vector4d& value, const auto& coord) {
      points.emplace_back(value.head<3>().cast<float>());
      intensities.emplace_back(value.w());
    });

    auto viewer = guik::LightViewer::instance();
    auto cloud_buffer = std::make_shared<glk::PointCloudBuffer>(points);
    cloud_buffer->add_intensity(glk::COLORMAP::TURBO, intensities, 1.0f / (*std::max_element(intensities.begin(), intensities.end())));
    viewer->update_drawable("points", cloud_buffer, guik::VertexColor());
    viewer->spin_once();
    */
  }
}

Frame::ConstPtr DynamicPointCloudIntegrator::get_points() {
  alignment_results.submit_end_of_data();
  voxelgrid_thread.join();

  std::vector<Eigen::Vector3f> points;
  std::vector<float> intensities;

  points.reserve(voxelgrid.size());
  intensities.reserve(voxelgrid.size());

  for (const auto& voxel : voxelgrid) {
    points.emplace_back(voxel.second.cast<float>().head<3>());
    intensities.emplace_back(voxel.second.w());
  }

  auto frame = std::make_shared<FrameCPU>(points);
  frame->add_intensities(intensities);
  return frame;
}

}  // namespace vlcal
