#include <random>
#include <chrono>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Eigen>

#include <opencv2/opencv.hpp>
#include <vlcal/calib/nearest_neighbor_search.hpp>
#include <vlcal/calib/line_fitting.hpp>

cv::Mat edges;

void on_mouse(int event, int x, int y, int flags, void* userdata) {
  auto nn = reinterpret_cast<vlcal::NearestNeighborSearch*>(userdata);

  if (event == cv::EVENT_LBUTTONDOWN) {
    cv::Mat canvas;
    cv::cvtColor(edges, canvas, cv::COLOR_GRAY2BGR);

    const int k = 10;
    std::vector<Eigen::Vector2i> k_neighbors(k);
    int num_found = nn->knn_search(Eigen::Vector2d(x, y), k, k_neighbors.data());
    k_neighbors.resize(num_found);

    std::cout << "---" << std::endl;
    for (int i = 0; i < num_found; i++) {
      std::cout << i << ":" << k_neighbors[i].transpose() << " dist:" << (k_neighbors[i] - Eigen::Vector2i(x, y)).norm() << std::endl;
      auto& bgr = canvas.at<cv::Vec3b>(k_neighbors[i].y(), k_neighbors[i].x());
      bgr[0] = 0;
      bgr[1] = 0;
      bgr[2] = 255;
    }

    if (num_found > 5) {
      const auto [p0, n] = vlcal::fit_line(k_neighbors);

      Eigen::Vector2d dir = Eigen::Rotation2Dd(M_PI_2) * n;

      Eigen::Vector2d pt0 = p0 - 20.0 * n;
      Eigen::Vector2d pt1 = p0 + 20.0 * n;

      cv::line(canvas, cv::Point(pt0.x(), pt0.y()), cv::Point(pt1.x(), pt1.y()), cv::Scalar(255, 0, 0));

      std::cout << "p0:" << p0.transpose() << std::endl;
      std::cout << "n :" << n.transpose() << std::endl;
    }

    cv::imshow("edges", canvas);
  }
}

void test(const cv::Mat& edges) {
  std::mt19937 mt;

  std::vector<Eigen::Vector2d> points(8192 * 10);
  for (int i = 0; i < points.size(); i++) {
    points[i].x() = std::uniform_real_distribution<>(0, edges.cols)(mt);
    points[i].y() = std::uniform_real_distribution<>(0, edges.rows)(mt);
  }

  const int k = 10;
  std::vector<Eigen::Vector2i> k_neighbors(k);

  int sum_found = 0;
  vlcal::ImageSpaceNearestNeighborSearch nn_i(edges, 10);

  auto t1 = std::chrono::high_resolution_clock::now();
  for (const auto& pt : points) {
    sum_found += nn_i.knn_search(pt, k, k_neighbors.data());
  }
  auto t2 = std::chrono::high_resolution_clock::now();

  std::cout << "i:" << sum_found << " time:" << std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count() / 1e6 << "[msec]" << std::endl;

  sum_found = 0;
  vlcal::KdTreeNearestNeighborSearch nn_k(edges);

  t1 = std::chrono::high_resolution_clock::now();
  for (const auto& pt : points) {
    sum_found += nn_k.knn_search(pt, k, k_neighbors.data());
  }
  t2 = std::chrono::high_resolution_clock::now();

  std::cout << "k:" << sum_found << " time:" << std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count() / 1e6 << "[msec]" << std::endl;
}

int main(int argc, char** argv) {
  cv::Mat image = cv::imread("/home/koide/datasets/lidar_camera/0.png", 0);
  cv::resize(image.clone(), image, cv::Size(0, 0), 0.5, 0.5);

  cv::equalizeHist(image.clone(), image);
  cv::Canny(image, edges, 100, 200);

  test(edges);

  vlcal::ImageSpaceNearestNeighborSearch nn(edges, 20);
  // vlcal::KdTreeNearestNeighborSearch nn(edges);
  cv::imshow("edges", edges);

  cv::setMouseCallback("edges", on_mouse, &nn);

  while (cv::waitKey(10) != 0x1b) {
  }

  /*
  auto viewer = guik::LightViewer::instance();

  auto ply = glk::load_ply("/home/koide/datasets/lidar_camera/0.ply");
  auto points = std::make_shared<gtsam_ext::FrameCPU>(ply->vertices);
  points = gtsam_ext::voxelgrid_sampling(points, 0.05);

  vlcal::DepthContinuousEdgeExtractionParams params;
  vlcal::DepthContinuousEdgeExtraction edge_extraction(params);
  edge_extraction.extract(points);

  viewer->spin();
  */

  return 0;
}