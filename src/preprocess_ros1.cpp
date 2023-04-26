#include <vlcal/preprocess/preprocess.hpp>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>

#define GLIM_ROS2
#include <vlcal/common/console_colors.hpp>
#include <vlcal/common/ros_cloud_converter.hpp>

namespace vlcal {

class PointCloudReaderROS1 : public PointCloudReader {
public:
  PointCloudReaderROS1(const std::string& bag_filename, const std::string& points_topic, const std::string& intensity_channel) : intensity_channel(intensity_channel) {
    reader.open(bag_filename);
    view = std::make_shared<rosbag::View>(reader, rosbag::TopicQuery(points_topic));

    msg_itr = view->begin();
  }

  virtual RawPoints::Ptr read_next() override {
    if (msg_itr == view->end()) {
      return nullptr;
    }

    const auto points_msg = msg_itr->instantiate<sensor_msgs::PointCloud2>();
    if (!points_msg) {
      return nullptr;
    }

    msg_itr++;
    return extract_raw_points(points_msg, intensity_channel);
  }

private:
  const std::string intensity_channel;

  rosbag::Bag reader;
  std::shared_ptr<rosbag::View> view;

  rosbag::View::iterator msg_itr;
};

class PreprocessROS1 : public Preprocess {
protected:
  template <typename T>
  boost::shared_ptr<T> get_first_message(const std::string& bag_filename, const std::string& topic) const {
    rosbag::Bag bag(bag_filename);
    rosbag::View view(bag, rosbag::TopicQuery(topic));

    for (const auto m : view) {
      const auto msg = m.instantiate<T>();
      if (msg) {
        return msg;
      }
    }

    std::cerr << console::yellow;
    std::cerr << "warning: bag does not contain topic" << std::endl;
    std::cerr << "       : bag_filename=" << bag_filename << std::endl;
    std::cerr << "       : topic=" << topic << std::endl;
    std::cerr << console::reset;

    return nullptr;
  }

  virtual bool valid_bag(const std::string& bag_filename) override {
    rosbag::Bag bag(bag_filename);
    return bag.isOpen();
  }

  virtual std::vector<std::pair<std::string, std::string>> get_topics_and_types(const std::string& bag_filename) override {
    rosbag::Bag bag(bag_filename);
    rosbag::View view(bag);

    std::vector<std::pair<std::string, std::string>> topics_and_types;
    for (const auto& conn : view.getConnections()) {
      topics_and_types.emplace_back(conn->topic, conn->datatype);
    }

    return topics_and_types;
  }

  virtual std::vector<std::string> get_point_fields(const std::string& bag_filename, const std::string& points_topic) override {
    const auto msg = get_first_message<sensor_msgs::PointCloud2>(bag_filename, points_topic);
    std::vector<std::string> fields(msg->fields.size());
    std::transform(msg->fields.begin(), msg->fields.end(), fields.begin(), [](const auto& field) { return field.name; });
    return fields;
  }

  virtual cv::Size get_image_size(const std::string& bag_filename, const std::string& image_topic) override {
    const auto image_msg = get_first_message<sensor_msgs::Image>(bag_filename, image_topic);
    if(image_msg) {
      return cv::Size(image_msg->width, image_msg->height);
    }

    const auto compressed_image_msg = get_first_message<sensor_msgs::CompressedImage>(bag_filename, image_topic);
    if (compressed_image_msg) {
      const cv::Mat image = cv_bridge::toCvCopy(*compressed_image_msg, "mono8")->image;
      return cv::Size(image.cols, image.rows);
    }

    std::cerr << console::bold_red << "error: failed to retrieve image messages from the bag file" << console::reset << std::endl;
    return cv::Size(0, 0);
  }

  virtual std::tuple<std::string, std::vector<double>, std::vector<double>> get_camera_info(const std::string& bag_filename, const std::string& camera_info_topic) override {
    const auto camera_info_msg = get_first_message<sensor_msgs::CameraInfo>(bag_filename, camera_info_topic);
    std::vector<double> intrinsic(4);
    intrinsic[0] = camera_info_msg->K[0];
    intrinsic[1] = camera_info_msg->K[4];
    intrinsic[2] = camera_info_msg->K[2];
    intrinsic[3] = camera_info_msg->K[5];
    std::vector<double> distortion_coeffs = camera_info_msg->D;

    return {camera_info_msg->distortion_model, intrinsic, distortion_coeffs};
  }

  virtual cv::Mat get_image(const std::string& bag_filename, const std::string& image_topic) override {
    const auto image_msg = get_first_message<sensor_msgs::Image>(bag_filename, image_topic);
    if (image_msg) {
      return cv_bridge::toCvCopy(*image_msg, "mono8")->image;
    }

    const auto compressed_image_msg = get_first_message<sensor_msgs::CompressedImage>(bag_filename, image_topic);
    if (compressed_image_msg) {
      return cv_bridge::toCvCopy(*compressed_image_msg, "mono8")->image;
    }

    std::cerr << console::bold_red << "error: failed to retrieve image messages from the bag file" << console::reset << std::endl;
    return cv::Mat();
  }

  virtual std::shared_ptr<PointCloudReader> get_point_cloud_reader(const std::string& bag_filename, const std::string& points_topic, const std::string& intensity_channel)
    override {
    return std::make_shared<PointCloudReaderROS1>(bag_filename, points_topic, intensity_channel);
  }
};

}  // namespace vlcal

int main(int argc, char** argv) {
  vlcal::PreprocessROS1 preprocess;
  preprocess.run(argc, argv);

  return 0;
}
