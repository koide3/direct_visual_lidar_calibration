#include <iostream>
#include <unordered_map>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <gtsam_ext/types/frame_cpu.hpp>
#include <gtsam_ext/util/vector3i_hash.hpp>

#include <glim/util/ros_cloud_converter.hpp>

#include <glk/io/ply_io.hpp>
#include <glk/pointcloud_buffer.hpp>
#include <guik/viewer/light_viewer.hpp>

class IntensityNormalizer {
public:
  IntensityNormalizer() {}

  void add(const double distance, const double intensity) {
    distances.emplace_back(distance);
    intensities.emplace_back(intensity);
  }

private:
  std::vector<double> distances;
  std::vector<double> intensities;
};

int main(int argc, char** argv) {
  /*
  const std::string bag_path = "/home/koide/datasets/lidar_camera/livox_data/2022-09-08-20-41-08.bag";

  rosbag::Bag bag(bag_path);
  if(!bag.isOpen()) {
    std::cerr << "failed to open " << bag_path << std::endl;
  }

  const double resolution = 0.02;
  std::unordered_map<Eigen::Vector3i, Eigen::Vector4d, gtsam_ext::Vector3iHash> voxelmap;

  for(rosbag::MessageInstance const m : rosbag::View(bag, rosbag::TopicQuery("/livox/points"))) {
    const auto points_msg = m.instantiate<sensor_msgs::PointCloud2>();
    const auto raw_points = glim::extract_raw_points(points_msg);

    for (int i = 0; i < raw_points->size(); i++) {
      const Eigen::Vector3i coord = (raw_points->points[i] / resolution).array().floor().cast<int>().head<3>();
      const Eigen::Vector4d xyzi(raw_points->points[i].x(), raw_points->points[i].y(), raw_points->points[i].z(), raw_points->intensities[i]);
      voxelmap[coord] = xyzi;
    }
  }

  glk::PLYData ply;
  for(const auto& voxel: voxelmap) {
    ply.vertices.emplace_back(voxel.second.cast<float>().head<3>());
    ply.intensities.emplace_back(voxel.second.w());
  }

  glk::save_ply_binary("/home/koide/points.ply", ply);

  auto viewer = guik::LightViewer::instance();
  auto cloud_buffer = std::make_shared<glk::PointCloudBuffer>(ply.vertices);
  cloud_buffer->add_intensity(glk::COLORMAP::TURBO, ply.intensities, 1.0 / *std::max_element(ply.intensities.begin(), ply.intensities.end()));
  viewer->update_drawable("points", cloud_buffer, guik::VertexColor());
  viewer->spin();
  */

  auto ply = glk::load_ply("/home/koide/points.ply");

  std::ofstream ofs("/home/koide/intensities.txt");
  for (int i = 0; i<ply->vertices.size(); i++) {
    ofs << ply->vertices[i].transpose() << " " << ply->intensities[i] << std::endl;
  }

  return 0;
}