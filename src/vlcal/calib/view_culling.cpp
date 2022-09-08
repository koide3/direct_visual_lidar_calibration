#include <vlcal/calib/view_culling.hpp>

#include <opencv2/core.hpp>
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

gtsam_ext::FrameCPU::Ptr ViewCulling::cull(const gtsam_ext::Frame::ConstPtr& points, const Eigen::Isometry3d& T_camera_lidar) const {
  std::vector<int> point_indices(points->size());
  std::vector<Eigen::Vector4d> points_camera(points->size());

  for (int i = 0; i < points->size(); i++) {
    point_indices[i] = i;
    points_camera[i] = T_camera_lidar * points->points[i];
  }

  point_indices = view_culling(point_indices, points_camera);
  for(int i=0; i<point_indices.size(); i++) {
    points_camera[i] = points_camera[point_indices[i]];
  }
  points_camera.resize(point_indices.size());

  return gtsam_ext::sample(points, point_indices);
}

std::vector<int> ViewCulling::view_culling(const std::vector<int>& point_indices, const std::vector<Eigen::Vector4d>& points_camera) const {
  std::vector<int> indices;
  indices.reserve(points_camera.size());

  cv::Mat dist_map(image_size.y(), image_size.x(), CV_64FC1, cv::Scalar::all(std::numeric_limits<double>::max()));
  cv::Mat index_map(image_size.y(), image_size.x(), CV_32SC1, cv::Scalar::all(-1));

  for (int i = 0; i < points_camera.size(); i++) {
    const auto& pt_camera = points_camera[i];
    const Eigen::Vector3d normalized_pt_camera = pt_camera.normalized().head<3>();
    if (normalized_pt_camera.z() < min_z) {
      continue;
    }

    const Eigen::Vector2i pt_2d = proj->project(pt_camera.head<3>()).cast<int>();
    if ((pt_2d.array() < Eigen::Array2i::Zero()).any() || (pt_2d.array() >= image_size.array()).any()) {
      continue;
    }

    indices.emplace_back(point_indices[i]);

    if (params.enable_depth_buffer_culling) {
      const double sq_dist = normalized_pt_camera.squaredNorm();
      if(sq_dist > dist_map.at<double>(pt_2d.y(), pt_2d.x())) {
        continue;
      }

      dist_map.at<double>(pt_2d.y(), pt_2d.x()) = sq_dist;
      index_map.at<int>(pt_2d.y(), pt_2d.x()) = point_indices[i];
    }
  }


  if(params.enable_depth_buffer_culling) {
    indices.clear();
    for (int i = 0; i < index_map.rows * index_map.cols; i++) {
      const int index = index_map.at<int>(i);
      if (index >= 0) {
        indices.emplace_back(index);
      }
    }
  }

  return indices;
}

/*
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
