#include <vlcal/calib/edge_extraction.hpp>

#include <bonxai/bonxai.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <glk/pointcloud_buffer.hpp>
#include <guik/viewer/light_viewer.hpp>

namespace vlcal {

DepthContinuousEdgeExtractionParams::DepthContinuousEdgeExtractionParams() {
  voxel_resolution = 1.0;

  min_voxel_points = 256;
  min_plane_points = 100;

  ransac_max_iterations = 5000;
  ransac_dist_thresh = 0.025;

  plane_angle_min = 45.0 * M_PI / 180.0;
  plane_angle_max = 135.0 * M_PI / 180.0;

  edge_sampling_step = 0.02;
}

DepthContinuousEdgeExtractionParams::~DepthContinuousEdgeExtractionParams() {}

struct DepthContinuousEdgeExtractionVoxel {
public:
  using Ptr = std::shared_ptr<DepthContinuousEdgeExtractionVoxel>;

  DepthContinuousEdgeExtractionVoxel() {
    points = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    points->reserve(128);
  }

  void insert(const Eigen::Vector4d& pt) { points->emplace_back(pcl::PointXYZ(pt.x(), pt.y(), pt.z())); }

  std::vector<Eigen::Vector4d> extract(const DepthContinuousEdgeExtractionParams& params) {
    if (points->size() < params.min_voxel_points) {
      return std::vector<Eigen::Vector4d>();
    }

    const auto planes = extract_planes(params);
    if (planes.size() > 3) {
      // Too many planes. Maybe wrong plane detections.
      return std::vector<Eigen::Vector4d>();
    }

    return extract_edges(planes, params);
  }

  /**
   * @brief
   * @param params
   * @return Plane parameters ax+by+cz+d=0
   */
  std::vector<Eigen::Vector4d> extract_planes(const DepthContinuousEdgeExtractionParams& params) {
    auto cloud = points;

    std::vector<Eigen::Vector4d> planes;
    while (cloud->size() > params.min_plane_points) {
      auto plane_model = pcl::make_shared<pcl::SampleConsensusModelPlane<pcl::PointXYZ>>(cloud);
      pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(plane_model);
      ransac.setMaxIterations(params.ransac_max_iterations);
      ransac.setDistanceThreshold(params.ransac_dist_thresh);
      ransac.computeModel();
      ransac.refineModel(1.0, 128);

      auto inliers = pcl::make_shared<pcl::Indices>();
      ransac.getInliers(*inliers);

      if (inliers->size() < params.min_plane_points) {
        break;
      }

      Eigen::VectorXf coefficients;
      ransac.getModelCoefficients(coefficients);
      planes.emplace_back(coefficients.cast<double>());

      pcl::ExtractIndices<pcl::PointXYZ> extract;
      extract.setInputCloud(cloud);
      extract.setIndices(inliers);

      extract.setNegative(false);
      plane_points.emplace_back(pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>());
      extract.filter(*plane_points.back());

      extract.setNegative(true);
      auto filtered = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
      extract.filter(*filtered);
      cloud = filtered;
    }

    return planes;
  }

  std::vector<Eigen::Vector4d> extract_edges(const std::vector<Eigen::Vector4d>& planes, const DepthContinuousEdgeExtractionParams& params) {
    Eigen::Vector3d min_pt = points->at(0).getVector3fMap().cast<double>();
    Eigen::Vector3d max_pt = points->at(0).getVector3fMap().cast<double>();

    for (const auto& pt : *points) {
      min_pt = min_pt.array().min(pt.getArray3fMap().cast<double>());
      max_pt = max_pt.array().max(pt.getArray3fMap().cast<double>());
    }

    // https://gist.github.com/DomNomNom/46bb1ce47f68d255fd5d
    const auto intersect_aabb_line = [&](const Eigen::Vector3d& p0, const Eigen::Vector3d& n) {
      const Eigen::Vector3d tmin = (min_pt - p0).array() / n.array();
      const Eigen::Vector3d tmax = (max_pt - p0).array() / n.array();
      const Eigen::Vector3d t1 = tmin.array().min(tmax.array());
      const Eigen::Vector3d t2 = tmin.array().max(tmax.array());
      const double tnear = t1.maxCoeff();
      const double tfar = t2.minCoeff();
      return Eigen::Vector2d(tnear, tfar);
    };

    std::vector<Eigen::Vector4d> edge_points;
    for (int i = 0; i < planes.size(); i++) {
      const auto& plane_a = planes[i];
      const Eigen::Vector3d normal_a = plane_a.head<3>().normalized();

      for (int j = i + 1; j < planes.size(); j++) {
        const auto& plane_b = planes[j];
        Eigen::Vector3d normal_b = plane_b.head<3>().normalized();
        if (normal_a.dot(normal_b) < 0.0) {
          normal_b = -normal_b;
        }

        const double angle = std::acos(normal_a.dot(normal_b));
        if (angle < params.plane_angle_min || angle > params.plane_angle_max) {
          // parallel planes
          continue;
        }

        const Eigen::Vector3d n = normal_a.cross(normal_b).normalized();
        const Eigen::Vector3d abs_n = n.array().abs();

        Eigen::Vector3d p0;

        if (abs_n.x() > abs_n.y() && abs_n.x() > abs_n.z()) {
          const Eigen::Matrix2d A = (Eigen::Matrix2d() << plane_a.y(), plane_a.z(), plane_b.y(), plane_b.z()).finished();
          const Eigen::Vector2d yz0 = A.inverse() * Eigen::Vector2d(-plane_a.w(), -plane_b.w());
          p0 << 0.0, yz0;
        } else if (abs_n.y() > abs_n.z()) {
          const Eigen::Matrix2d A = (Eigen::Matrix2d() << plane_a.x(), plane_a.z(), plane_b.x(), plane_b.z()).finished();
          const Eigen::Vector2d xz0 = A.inverse() * Eigen::Vector2d(-plane_a.w(), -plane_b.w());
          p0 << xz0[0], 0.0, xz0[1];
        } else {
          const Eigen::Matrix2d A = (Eigen::Matrix2d() << plane_a.x(), plane_a.y(), plane_b.x(), plane_b.y()).finished();
          const Eigen::Vector2d xy0 = A.inverse() * Eigen::Vector2d(-plane_a.w(), -plane_b.w());
          p0 << xy0, 0.0;
        }

        p0 -= 100.0 * n;

        const Eigen::Vector2d ts = intersect_aabb_line(p0, n);
        if (ts.x() > ts.y()) {
          // no line-aabb intersection
          continue;
        }

        // sample points along the intersection line
        for (double t = ts[0]; t <= ts[1]; t += params.edge_sampling_step) {
          edge_points.emplace_back((Eigen::Vector4d() << p0 + t * n, 1.0).finished());
        }
      }
    }

    return edge_points;
  }

public:
  pcl::PointCloud<pcl::PointXYZ>::Ptr points;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> plane_points;
};

DepthContinuousEdgeExtraction::DepthContinuousEdgeExtraction(const DepthContinuousEdgeExtractionParams& params) : params(params) {}

DepthContinuousEdgeExtraction::~DepthContinuousEdgeExtraction() {}

gtsam_ext::FrameCPU::Ptr DepthContinuousEdgeExtraction::extract(const gtsam_ext::Frame::ConstPtr& points, std::vector<gtsam_ext::FrameCPU::Ptr>* plane_points) {
  Bonxai::VoxelGrid<DepthContinuousEdgeExtractionVoxel::Ptr> voxelgrid(params.voxel_resolution);
  auto accessor = voxelgrid.createAccessor();

  for (int i = 0; i < points->size(); i++) {
    const auto& pt = points->points[i];

    const auto coord = voxelgrid.posToCoord(pt.x(), pt.y(), pt.z());

    DepthContinuousEdgeExtractionVoxel* voxel;

    auto found = accessor.value(coord);
    if (found == nullptr) {
      auto new_voxel = std::make_shared<DepthContinuousEdgeExtractionVoxel>();
      accessor.setValue(coord, new_voxel);
      voxel = new_voxel.get();
    } else {
      voxel = found->get();
    }

    voxel->insert(pt);
  }

  std::vector<DepthContinuousEdgeExtractionVoxel*> voxels;
  voxelgrid.forEachCell([&](const auto& voxel, const auto& coord) { voxels.emplace_back(voxel.get()); });

  std::vector<Eigen::Vector4d> edge_points;

#pragma omp parallel for schedule(guided, 4)
  for (int i = 0; i < voxels.size(); i++) {
    auto voxel = voxels[i];
    const auto new_points = voxel->extract(params);
    if (new_points.empty()) {
      continue;
    }

#pragma critical
    {
      edge_points.insert(edge_points.end(), new_points.begin(), new_points.end());
      if (plane_points) {
        for (const auto& pts : voxel->plane_points) {
          auto new_points = std::make_shared<gtsam_ext::FrameCPU>();
          new_points->num_points = pts->size();
          new_points->points_storage.resize(pts->size());
          new_points->points = new_points->points_storage.data();
          std::transform(pts->begin(), pts->end(), new_points->points, [](const auto& pt) { return pt.getVector4fMap().template cast<double>(); });
          plane_points->emplace_back(new_points);
        }
      }
    }
  }

  return std::make_shared<gtsam_ext::FrameCPU>(edge_points);
}

}  // namespace vlcal
