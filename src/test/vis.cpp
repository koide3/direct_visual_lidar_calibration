#include <iostream>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>

#include <glim/util/ros_cloud_converter.hpp>

#include <glk/io/ply_io.hpp>
#include <glk/pointcloud_buffer.hpp>
#include <guik/viewer/light_viewer.hpp>

int main(int argc, char** argv) {
  rosbag::Bag bag("/home/koide/datasets/lidar_camera/ouster_data/00_2022-09-05-15-36-26.bag");
  if (!bag.isOpen()) {
    std::cerr << "error: failed to open the rosbag!!" << std::endl;
    return 1;
  }

  auto viewer = guik::LightViewer::instance();
  viewer->set_draw_xy_grid(false);

  const auto ply = glk::load_ply("/home/koide/datasets/lidar_camera/ouster_pinhole_all/00_2022-09-05-15-36-26.bag.ply");
  auto cloud_buffer = std::make_shared<glk::PointCloudBuffer>(ply->vertices);
  cloud_buffer->add_intensity(glk::COLORMAP::TURBO, ply->intensities);
  viewer->update_drawable("dense", cloud_buffer, guik::VertexColor());

  for (rosbag::MessageInstance const m : rosbag::View(bag, rosbag::TopicQuery("/os_cloud_node/points"))) {
    const auto points_msg = m.instantiate<sensor_msgs::PointCloud2>();
    auto raw_points = glim::extract_raw_points(points_msg, "reflectivity");

    const double max_intensity = *std::max_element(raw_points->intensities.begin(), raw_points->intensities.end());
    auto cloud_buffer = std::make_shared<glk::PointCloudBuffer>(raw_points->points);
    cloud_buffer->add_intensity(glk::COLORMAP::TURBO, raw_points->intensities, 10.0 / max_intensity);
    viewer->update_drawable("sparse", cloud_buffer, guik::VertexColor());
    viewer->spin_until_click();
  }

  return 0;
}