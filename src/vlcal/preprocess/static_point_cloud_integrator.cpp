#include <vlcal/preprocess/static_point_cloud_integrator.hpp>

#include <glk/pointcloud_buffer.hpp>
#include <guik/viewer/light_viewer.hpp>

namespace vlcal {

StaticPointCloudIntegratorParams::StaticPointCloudIntegratorParams() {
  visualize = false;
  voxel_resolution = 0.05;
  min_distance = 1.0;
}

StaticPointCloudIntegratorParams::~StaticPointCloudIntegratorParams() {}

StaticPointCloudIntegrator::StaticPointCloudIntegrator(const StaticPointCloudIntegratorParams& params) : params(params) {
  if (params.visualize) {
    auto viewer = guik::LightViewer::instance();
    viewer->clear_drawables();
  }
}

StaticPointCloudIntegrator::~StaticPointCloudIntegrator() {}

void StaticPointCloudIntegrator::insert_points(const Frame::ConstPtr& raw_points) {
  for (int i = 0; i < raw_points->size(); i++) {
    const auto& pt = raw_points->points[i];
    const double intensity = raw_points->intensities[i];

    if (pt.head<3>().norm() < params.min_distance) {
      continue;
    }

    const Eigen::Vector3i coord = (pt / params.voxel_resolution).array().floor().cast<int>().head<3>();
    voxelgrid[coord] = Eigen::Vector4d(pt[0], pt[1], pt[2], intensity);
  }

  if (params.visualize) {
    auto viewer = guik::LightViewer::instance();
    auto cloud_buffer = std::make_shared<glk::PointCloudBuffer>(raw_points->points, raw_points->size());
    viewer->update_drawable(guik::anon(), cloud_buffer, guik::Rainbow());
    viewer->update_drawable("current", cloud_buffer, guik::FlatOrange().set_point_scale(2.0f));
    viewer->spin_once();
  }
}

Frame::ConstPtr StaticPointCloudIntegrator::get_points() {
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