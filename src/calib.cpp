#include <iostream>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <gtsam_ext/types/frame_cpu.hpp>
#include <glim/util/time_keeper.hpp>
#include <glim/util/ros_cloud_converter.hpp>

#include <glk/pointcloud_buffer.hpp>
#include <glk/primitives/primitives.hpp>
#include <guik/viewer/light_viewer.hpp>

#include <vlcal/preprocess/point_cloud_integrator.hpp>

int main(int argc, char** argv) {
  const std::string bag_path = "/home/koide/datasets/drone_color/mapping2/mapping_12.bag";
  const std::string points_topic = "/os_cloud_node/points";

  glim::TimeKeeper time_keeper;

  vlcal::PointCloudIntegrator integrator;

  rosbag::Bag bag(bag_path);
  for(rosbag::MessageInstance const m : rosbag::View(bag, rosbag::TopicQuery(points_topic))) {
    const auto points_msg = m.instantiate<sensor_msgs::PointCloud2>();
    auto raw_points = glim::extract_raw_points(points_msg, "reflectivity");
    time_keeper.process(raw_points);

    auto points = std::make_shared<gtsam_ext::FrameCPU>(raw_points->points);
    points->add_times(raw_points->times);
    points->add_intensities(raw_points->intensities);
    points = gtsam_ext::sort_by_time(points);

    integrator.insert_points(points);
  }

  return 0;
}