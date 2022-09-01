#pragma once

#include <random>
#include <thread>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gtsam_ext/types/frame_cpu.hpp>
#include <vlcal/common/concurrent_queue.hpp>
#include <vlcal/preprocess/point_cloud_integrator.hpp>

namespace Bonxai {
template <typename T>
class VoxelGrid;
}

namespace vlcal {

struct StaticPointCloudIntegratorParams {
public:
  StaticPointCloudIntegratorParams();
  ~StaticPointCloudIntegratorParams();

  double voxel_resolution;
};

class StaticPointCloudIntegrator : public PointCloudIntegrator {
public:
  StaticPointCloudIntegrator(const StaticPointCloudIntegratorParams& params = StaticPointCloudIntegratorParams());
  ~StaticPointCloudIntegrator();

  virtual void insert_points(const gtsam_ext::Frame::ConstPtr& raw_points) override;
  virtual gtsam_ext::Frame::ConstPtr get_points() override;

private:
  const StaticPointCloudIntegratorParams params;

  std::unique_ptr<Bonxai::VoxelGrid<Eigen::Vector4d>> voxelgrid;
};
}