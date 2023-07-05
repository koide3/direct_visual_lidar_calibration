#include <vlcal/common/estimate_fov.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <dfo/nelder_mead.hpp>
#include <dfo/directional_direct_search.hpp>

#include <vlcal/common/frame_cpu.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/voxel_grid.h>

namespace vlcal {

Eigen::Vector3d estimate_direction(const camera::GenericCameraBase::ConstPtr& proj, const Eigen::Vector2d& pt_2d) {
  const auto to_dir = [](const Eigen::Vector2d& x) {
    return Eigen::AngleAxisd(x[0], Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(x[1], Eigen::Vector3d::UnitY()) * Eigen::Vector3d::UnitZ();
  };

  const auto f = [&](const Eigen::Vector2d& x) {
    const Eigen::Vector3d dir = to_dir(x);
    const double err = (pt_2d - proj->project(dir)).squaredNorm();
    return std::isfinite(err) ? err : std::numeric_limits<double>::max();
  };

  // TODO : should use differential-based optimization
  dfo::NelderMead<2>::Params params;
  dfo::NelderMead<2> optimizer(params);
  auto result = optimizer.optimize(f, Eigen::Vector2d::Zero());

  return to_dir(result.x);
}

double estimate_camera_fov(const camera::GenericCameraBase::ConstPtr& proj, const Eigen::Vector2i& image_size) {
  const std::vector<Eigen::Vector2d> target_corners = {Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(image_size[0] / 2, 0.0), Eigen::Vector2d(0.0, image_size[1] / 2)};

  // Transform points into the bearing vectors and find the maximum view angle from them
  double max_fov = 0.0;
  for (const auto& corner : target_corners) {
    const auto dir = estimate_direction(proj, corner);
    const double fov = std::acos(dir.normalized().z());

    if (fov > max_fov) {
      max_fov = fov;
    }
  }

  return max_fov;
}

double estimate_lidar_fov(const Frame::ConstPtr& points) {
  auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  cloud->resize(points->size());
  std::transform(points->points, points->points + points->size(), cloud->begin(), [](const auto& p) { return pcl::PointXYZ(p.x(), p.y(), p.z()); });

  auto filtered = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
  voxelgrid.setLeafSize(0.2f, 0.2f, 0.2f);
  voxelgrid.setInputCloud(cloud);
  voxelgrid.filter(*filtered);

  const auto remove_loc = std::remove_if(filtered->begin(), filtered->end(), [](const pcl::PointXYZ& pt) { return pt.getVector3fMap().norm() < 1.0; });
  filtered->erase(remove_loc, filtered->end());
  cloud = filtered;

  // convexhull
  pcl::ConvexHull<pcl::PointXYZ> convexhull;
  convexhull.setInputCloud(cloud);

  auto hull = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  convexhull.reconstruct(*hull);

  // Precompute bearing vectors
  std::vector<Eigen::Vector3d> dirs(hull->size());
  for (int i = 0; i < hull->size(); i++) {
    dirs[i] = hull->at(i).getVector3fMap().cast<double>().normalized();
  }

  // Find the maximum angle in the convexhull
  double min_cosine = M_PI;
  for (int i = 0; i < hull->size(); i++) {
    for (int j = i + 1; j < hull->size(); j++) {
      const double cosine = dirs[i].dot(dirs[j]);
      min_cosine = std::min(cosine, min_cosine);
    }
  }

  return std::acos(min_cosine);
}

}  // namespace vlcal
