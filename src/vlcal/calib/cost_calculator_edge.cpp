#include <vlcal/calib/cost_calculator_edge.hpp>

#include <iostream>
#include <opencv2/opencv.hpp>

#include <vlcal/common/estimate_fov.hpp>
#include <vlcal/calib/line_fitting.hpp>

#include <glk/pointcloud_buffer.hpp>
#include <guik/viewer/light_viewer.hpp>

namespace vlcal {

EdgeCostParams::EdgeCostParams() {
  num_threads = 1;

  init_max_correspondence_dist = 30.0;
  dec_max_correspondence_dist = 0.2;
  min_max_correspondence_dist = 5.0;
}

EdgeCostParams::~EdgeCostParams() {}

CostCalculatorEdge::CostCalculatorEdge(const camera::GenericCameraBase::ConstPtr& proj, const VisualLiDARData::ConstPtr& data, const EdgeCostParams& params)
: params(params),
  proj(proj),
  max_fov(estimate_camera_fov(proj, {data->image.cols, data->image.rows})),
  data(data) {
  DepthContinuousEdgeExtraction edge_edtraction(params.edge_params);
  auto downsampled = gtsam_ext::voxelgrid_sampling(data->points, 0.05);
  edges = edge_edtraction.extract(downsampled);
  
  cv::GaussianBlur(data->image, edge_image, cv::Size(5, 5), 3.0);
  cv::Canny(edge_image.clone(), edge_image, 100.0, 200.0);
  nn_search.reset(new KdTreeNearestNeighborSearch(edge_image));

  max_correspondence_dist = params.init_max_correspondence_dist;
}

CostCalculatorEdge::~CostCalculatorEdge() {}

void CostCalculatorEdge::update_correspondences(const Eigen::Isometry3d& T_camera_lidar) {
  const int k = 20;

  correspondences.clear();

  cv::Mat canvas;
  cv::cvtColor(edge_image, canvas, cv::COLOR_GRAY2BGR);

  for (int i = 0; i < edges->size(); i++) {
    const Eigen::Vector4d pt_camera = T_camera_lidar * edges->points[i];
    if (pt_camera.head<3>().normalized().z() < std::cos(max_fov)) {
      continue;
    }

    const Eigen::Vector2d pt_2d = proj->project(pt_camera.head<3>());
    if ((pt_2d.array() < Eigen::Array2d::Zero()).any() || (pt_2d.array() >= Eigen::Array2d(data->image.cols, data->image.rows)).any()) {
      continue;
    }

    std::vector<Eigen::Vector2i> k_neighbors(k);
    const int num_found = nn_search->knn_search(pt_2d, k, k_neighbors.data());
    k_neighbors.resize(num_found);

    auto remove_loc = std::remove_if(k_neighbors.begin(), k_neighbors.end(), [&](const Eigen::Vector2i& x) { return (pt_2d - x.cast<double>()).norm() > max_correspondence_dist; });
    k_neighbors.erase(remove_loc, k_neighbors.end());

    if(k_neighbors.size() < 5) {
      continue;
    }

    const auto [p0, n] = fit_line(k_neighbors);
    correspondences.emplace_back(std::make_pair(edges->points[i], (Eigen::Vector4d() << p0, n).finished()));

    cv::line(canvas, cv::Point(pt_2d.x(), pt_2d.y()), cv::Point(p0.x(), p0.y()), cv::Scalar(0, 255, 0));
  }

  max_correspondence_dist = std::max(params.min_max_correspondence_dist, max_correspondence_dist - params.dec_max_correspondence_dist);

  cv::imshow("canvas", canvas);
  cv::waitKey(1);
}

double CostCalculatorEdge::calculate(const Eigen::Isometry3d& T_camera_lidar) {
  if (correspondences.empty()) {
    update_correspondences(T_camera_lidar);
  }

  double sum_dists = 0.0;
  for (int i = 0; i < correspondences.size(); i++) {
    const auto& [pt_lidar, line_2d] = correspondences[i];
    const Eigen::Vector2d pt_2d = proj->project((T_camera_lidar * pt_lidar).head<3>());

    const auto& line_p0 = line_2d.head<2>();
    const auto& line_n = line_2d.tail<2>();

    sum_dists += std::pow((pt_2d - line_p0).dot(line_n), 2);
  }

  return sum_dists;
}

}  // namespace vlcal
