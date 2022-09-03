#include <iostream>

#include <glk/io/ply_io.hpp>
#include <glk/thin_lines.hpp>
#include <glk/pointcloud_buffer.hpp>
#include <glk/pointcloud_buffer_pcl.hpp>
#include <guik/viewer/light_viewer.hpp>

#include <gtsam_ext/types/frame_cpu.hpp>

#include <vlcal/calib/edge_extraction.hpp>

int main(int argc, char** argv) {
  auto viewer = guik::LightViewer::instance();

  auto ply = glk::load_ply("/home/koide/datasets/lidar_camera/0.ply");
  auto points = std::make_shared<gtsam_ext::FrameCPU>(ply->vertices);
  points = gtsam_ext::voxelgrid_sampling(points, 0.05);

  vlcal::DepthContinuousEdgeExtractionParams params;
  vlcal::DepthContinuousEdgeExtraction edge_extraction(params);
  edge_extraction.extract(points);

  viewer->spin();

  return 0;
}