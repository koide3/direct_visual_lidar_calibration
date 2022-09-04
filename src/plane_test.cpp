#include <iostream>

#include <glk/io/ply_io.hpp>
#include <glk/thin_lines.hpp>
#include <glk/pointcloud_buffer.hpp>
#include <glk/pointcloud_buffer_pcl.hpp>
#include <guik/viewer/light_viewer.hpp>

#include <gtsam_ext/types/frame_cpu.hpp>

#include <vlcal/calib/edge_extraction.hpp>

#include <opencv2/opencv.hpp>
#include <vlcal/calib/nearest_neighbor_search.hpp>

cv::Mat edges;

void on_mouse(int event, int x, int y, int flags, void* userdata) {
  auto nn = reinterpret_cast<vlcal::NearestNeighborSearch*>(userdata);

  if (event == cv::EVENT_LBUTTONDOWN) {
    cv::Mat canvas;
    cv::cvtColor(edges, canvas, cv::COLOR_GRAY2BGR);

    const int k = 20;
    std::vector<Eigen::Vector2i> k_neighbors(k);
    int num_found = nn->knn_search(Eigen::Vector2d(x, y), k, k_neighbors.data());

    std::cout << "---" << std::endl;
    for (int i = 0; i < num_found; i++) {
      std::cout << i << ":" << k_neighbors[i].transpose() << " dist:" << (k_neighbors[i] - Eigen::Vector2i(x, y)).norm() << std::endl;
      auto& bgr = canvas.at<cv::Vec3b>(k_neighbors[i].y(), k_neighbors[i].x());
      bgr[0] = 0;
      bgr[1] = 0;
      bgr[2] = 255;
    }

    cv::imshow("edges", canvas);
  }
}

int main(int argc, char** argv) {
  cv::Mat image = cv::imread("/home/koide/datasets/lidar_camera/0.png", 0);
  cv::resize(image.clone(), image, cv::Size(0, 0), 0.5, 0.5);

  cv::equalizeHist(image.clone(), image);
  cv::Canny(image, edges, 100, 200);

  // vlcal::ImageSpaceNearestNeighborSearch nn(edges, 20);
  vlcal::KdTreeNearestNeighborSearch nn(edges);
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