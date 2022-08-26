#pragma once

#include <random>
#include <thread>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gtsam/geometry/Pose3.h>
#include <gtsam_ext/types/frame_cpu.hpp>
#include <vlcal/common/concurrent_queue.hpp>

namespace gtsam_ext {
class iVox;
}

namespace Bonxai {
template <typename T>
class VoxelGrid;
}

namespace vlcal {

struct PointCloudIntegratorParams {
public:
  PointCloudIntegratorParams();
  ~PointCloudIntegratorParams();

  int num_threads;
  int k_neighbors;
  int target_num_points;

  double voxel_resolution;
};

class PointCloudIntegrator {
public:
  PointCloudIntegrator(const PointCloudIntegratorParams& params = PointCloudIntegratorParams());
  ~PointCloudIntegrator();

  void insert_points(const gtsam_ext::Frame::ConstPtr& raw_points);
  void insert_points(const gtsam_ext::Frame::ConstPtr& raw_points, const gtsam::Pose3& T_odom_lidar_begin, const gtsam::Pose3& T_odom_lidar_end);

  void voxelgrid_task();

private:
  const PointCloudIntegratorParams params;

  std::mt19937 mt;

  gtsam::Pose3 last_T_odom_lidar_begin;
  gtsam::Pose3 last_T_odom_lidar_end;
  std::shared_ptr<gtsam_ext::iVox> target_ivox;

  std::thread voxelgrid_thread;
  ConcurrentQueue<std::tuple<gtsam_ext::Frame::ConstPtr, gtsam::Pose3, gtsam::Pose3>> alignment_results;
  std::unique_ptr<Bonxai::VoxelGrid<Eigen::Vector4d>> voxelgrid;
};
}