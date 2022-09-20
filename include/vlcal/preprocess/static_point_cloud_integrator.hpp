#pragma once

#include <random>
#include <thread>
#include <unordered_map>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gtsam_ext/types/frame_cpu.hpp>
#include <gtsam_ext/util/vector3i_hash.hpp>
#include <vlcal/common/concurrent_queue.hpp>
#include <vlcal/preprocess/point_cloud_integrator.hpp>


namespace vlcal {

/**
 * @brief Static LiDAR point integrator for non-repetitive scan LiDARs
 */
struct StaticPointCloudIntegratorParams {
public:
  StaticPointCloudIntegratorParams();
  ~StaticPointCloudIntegratorParams();

  double voxel_resolution;
  double min_distance;
};

class StaticPointCloudIntegrator : public PointCloudIntegrator {
public:
  StaticPointCloudIntegrator(const StaticPointCloudIntegratorParams& params = StaticPointCloudIntegratorParams());
  ~StaticPointCloudIntegrator();

  virtual void insert_points(const gtsam_ext::Frame::ConstPtr& raw_points) override;
  virtual gtsam_ext::Frame::ConstPtr get_points() override;

private:
  const StaticPointCloudIntegratorParams params;

  std::unordered_map<Eigen::Vector3i, Eigen::Vector4d, gtsam_ext::Vector3iHash> voxelgrid;
};
}