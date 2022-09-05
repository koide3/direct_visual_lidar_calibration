#include <vlcal/preprocess/static_point_cloud_integrator.hpp>

namespace vlcal {

StaticPointCloudIntegratorParams::StaticPointCloudIntegratorParams() {
  voxel_resolution = 0.05;
  min_distance = 1.0;
}

StaticPointCloudIntegratorParams::~StaticPointCloudIntegratorParams() {}

StaticPointCloudIntegrator::StaticPointCloudIntegrator(const StaticPointCloudIntegratorParams& params) : params(params) {
}

StaticPointCloudIntegrator::~StaticPointCloudIntegrator() {}

void StaticPointCloudIntegrator::insert_points(const gtsam_ext::Frame::ConstPtr& raw_points) {
  for (int i = 0; i < raw_points->size(); i++) {
    const auto& pt = raw_points->points[i];
    const double intensity = raw_points->intensities[i];

    const Eigen::Vector3i coord = (pt / params.voxel_resolution).array().floor().cast<int>().head<3>();
    voxelgrid[coord] = Eigen::Vector4d(pt[0], pt[1], pt[2], intensity);
  }
}

gtsam_ext::Frame::ConstPtr StaticPointCloudIntegrator::get_points() {
  std::vector<Eigen::Vector3f> points;
  std::vector<float> intensities;

  points.reserve(voxelgrid.size());
  intensities.reserve(voxelgrid.size());

  for (const auto& voxel : voxelgrid) {
    points.emplace_back(voxel.second.cast<float>().head<3>());
    intensities.emplace_back(voxel.second.w());
  }

  auto frame = std::make_shared<gtsam_ext::FrameCPU>(points);
  frame->add_intensities(intensities);
  return frame;
}

}