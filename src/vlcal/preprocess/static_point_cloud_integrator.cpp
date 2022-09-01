#include <vlcal/preprocess/static_point_cloud_integrator.hpp>

#include <bonxai/bonxai.hpp>

namespace vlcal {

StaticPointCloudIntegratorParams::StaticPointCloudIntegratorParams() {
  voxel_resolution = 0.05;
}

StaticPointCloudIntegratorParams::~StaticPointCloudIntegratorParams() {}

StaticPointCloudIntegrator::StaticPointCloudIntegrator(const StaticPointCloudIntegratorParams& params) : params(params) {
  voxelgrid.reset(new Bonxai::VoxelGrid<Eigen::Vector4d>(params.voxel_resolution));
}

StaticPointCloudIntegrator::~StaticPointCloudIntegrator() {}

void StaticPointCloudIntegrator::insert_points(const gtsam_ext::Frame::ConstPtr& raw_points) {
  auto accessor = voxelgrid->createAccessor();

  for (int i = 0; i < raw_points->size(); i++) {
    const auto& pt = raw_points->points[i];
    const double intensity = raw_points->intensities[i];

    const Bonxai::CoordT coord = voxelgrid->posToCoord(pt[0], pt[1], pt[2]);
    accessor.setValue(coord, Eigen::Vector4d(pt[0], pt[1], pt[2], intensity));
  }
}

gtsam_ext::Frame::ConstPtr StaticPointCloudIntegrator::get_points() {
  std::vector<Eigen::Vector3f> points;
  std::vector<float> intensities;

  voxelgrid->forEachCell([&](const Eigen::Vector4d& value, const auto& coord) {
    points.emplace_back(value.head<3>().cast<float>());
    intensities.emplace_back(value.w());
  });

  auto frame = std::make_shared<gtsam_ext::FrameCPU>(points);
  frame->add_intensities(intensities);
  return frame;
}

}