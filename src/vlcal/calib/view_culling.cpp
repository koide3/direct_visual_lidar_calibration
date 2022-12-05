#include <vlcal/calib/view_culling.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <vlcal/common/estimate_fov.hpp>

// extern "C" {
// #include <libqhull_r/libqhull_r.h>
// }

namespace vlcal {

ViewCulling::ViewCulling(const camera::GenericCameraBase::ConstPtr& proj, const Eigen::Vector2i& image_size, const ViewCullingParams& params)
: params(params),
  proj(proj),
  image_size(image_size),
  min_z(std::cos(estimate_camera_fov(proj, image_size))) {}

ViewCulling ::~ViewCulling() {}

FrameCPU::Ptr ViewCulling::cull(const Frame::ConstPtr& points, const Eigen::Isometry3d& T_camera_lidar) const {
  std::vector<int> point_indices(points->size());
  std::vector<Eigen::Vector4d> points_camera(points->size());

  for (int i = 0; i < points->size(); i++) {
    point_indices[i] = i;
    points_camera[i] = T_camera_lidar * points->points[i];
  }

  point_indices = view_culling(point_indices, points_camera);
  return sample(points, point_indices);
}

std::vector<int> ViewCulling::view_culling(const std::vector<int>& point_indices, const std::vector<Eigen::Vector4d>& points_camera) const {
  std::vector<int> indices;
  std::vector<Eigen::Vector2i> projected_points;
  indices.reserve(points_camera.size());
  projected_points.reserve(points_camera.size());

  cv::Mat dist_map(image_size.y(), image_size.x(), CV_32FC1, cv::Scalar::all(std::numeric_limits<double>::max()));
  cv::Mat index_map(image_size.y(), image_size.x(), CV_32SC1, cv::Scalar::all(-1));

  for (int i = 0; i < points_camera.size(); i++) {
    const auto& pt_camera = points_camera[i];
    if (pt_camera.normalized().head<3>().z() < min_z) {
      // Out of FoV
      continue;
    }

    const Eigen::Vector2i pt_2d = proj->project(pt_camera.head<3>()).cast<int>();
    if ((pt_2d.array() < Eigen::Array2i::Zero()).any() || (pt_2d.array() >= image_size.array()).any()) {
      // Out of image
      continue;
    }

    indices.emplace_back(point_indices[i]);
    projected_points.emplace_back(pt_2d);

    if (params.enable_depth_buffer_culling) {
      const double dist = pt_camera.head<3>().norm();
      if (dist > dist_map.at<float>(pt_2d.y(), pt_2d.x())) {
        continue;
      }

      dist_map.at<float>(pt_2d.y(), pt_2d.x()) = dist;
      index_map.at<int>(pt_2d.y(), pt_2d.x()) = point_indices[i];
    }
  }

  if (params.enable_depth_buffer_culling) {
    std::vector<int> new_indices;
    new_indices.reserve(indices.size());

    for(int i=0; i<indices.size(); i++) {
      const auto index = indices[i];
      const auto& pt_2d = projected_points[i];

      const auto& pt_camera = points_camera[index];
      const double dist = pt_camera.head<3>().norm();

      if (dist > dist_map.at<float>(pt_2d.y(), pt_2d.x()) + 0.1) {
        continue;
      }

      new_indices.emplace_back(index);
    }

    indices = std::move(new_indices);
  }

  return indices;
}

/*
// Not as good as expected
std::vector<int> ViewCulling::hidden_points_removal(const std::vector<int>& point_indices, const std::vector<Eigen::Vector4d>& points_camera) const {
  // hidden points removal
  // [Katz 2007]
  std::vector<Eigen::Vector3d> flipped(points_camera.size() + 1);
  for (int i = 0; i < points_camera.size(); i++) {
    const auto& pt = points_camera[i];
    const double pt_norm = pt.head<3>().norm();
    flipped[i] = (pt + 2.0 * (params.hidden_points_removal_max_z - pt_norm) * pt / pt_norm).head<3>();
  }
  flipped.back().setZero();

  qhT qhull_handle;
  QHULL_LIB_CHECK
  qh_zero(&qhull_handle, stderr);

  char qhull_cmd[] = "qhull ";
  int code = qh_new_qhull(&qhull_handle, 3, flipped.size(), flipped[0].data(), false, qhull_cmd, nullptr, stderr);
  if (code) {
    std::cerr << "error: failed to compute convex hull" << std::endl;

    qh_freeqhull(&qhull_handle, !qh_ALL);
    int curlong, totlong;
    qh_memfreeshort(&qhull_handle, &curlong, &totlong);
    return point_indices;
  }

  std::vector<unsigned int> hull_indices(qhull_handle.num_vertices);
  auto hull_index_ptr = hull_indices.begin();
  for (vertexT* vertex = qhull_handle.vertex_list; vertex && vertex->next; vertex = vertex->next) {
    *(hull_index_ptr++) = qh_pointid(&qhull_handle, vertex->point);
  }
  auto found = std::find(hull_indices.begin(), hull_indices.end(), points_camera.size());
  if (found == hull_indices.end()) {
    std::cerr << "invalid!!" << std::endl;
  } else {
    hull_indices.erase(found);
  }

  auto min = std::min_element(hull_indices.begin(), hull_indices.end());
  auto max = std::max_element(hull_indices.begin(), hull_indices.end());

  std::vector<int> visible_indices(hull_indices.size());
  std::transform(hull_indices.begin(), hull_indices.end(), visible_indices.begin(), [&](const int i) { return point_indices[i]; });

  qh_freeqhull(&qhull_handle, !qh_ALL);
  int curlong, totlong;
  qh_memfreeshort(&qhull_handle, &curlong, &totlong);

  return visible_indices;
}
*/

}  // namespace vlcal
