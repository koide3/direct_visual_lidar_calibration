#pragma once

#include <string>
#include <vector>
#include <opencv2/core.hpp>
#include <boost/program_options.hpp>
#include <vlcal/common/frame.hpp>
#include <vlcal/common/raw_points.hpp>

namespace vlcal {

class PointCloudReader {
public:
  virtual ~PointCloudReader() {}
  virtual RawPoints::Ptr read_next() = 0;
};

class Preprocess {
public:
  Preprocess();
  ~Preprocess();

  bool run(int argc, char** argv);

protected:
  virtual bool valid_bag(const std::string& bag_filename) = 0;
  virtual std::vector<std::pair<std::string, std::string>> get_topics_and_types(const std::string& bag_filename) = 0;
  virtual std::vector<std::string> get_point_fields(const std::string& bag_filename, const std::string& points_topic) = 0;
  virtual cv::Size get_image_size(const std::string& bag_filename, const std::string& image_topic) = 0;
  virtual std::tuple<std::string, std::vector<double>, std::vector<double>> get_camera_info(const std::string& bag_filename, const std::string& camera_info_topic) = 0;
  virtual cv::Mat get_image(const std::string& bag_filename, const std::string& image_topic) = 0;
  virtual std::shared_ptr<PointCloudReader> get_point_cloud_reader(const std::string& bag_filename, const std::string& points_topic, const std::string& intensity_channel) = 0;

private:
  std::tuple<std::string, std::string, std::string> get_topics(const boost::program_options::variables_map& vm, const std::string& bag_filename);
  std::string get_intensity_channel(const boost::program_options::variables_map& vm, const std::string& bag_filename, const std::string& points_topic);
  std::tuple<std::string, cv::Size, std::vector<double>, std::vector<double>>
  get_camera_params(const boost::program_options::variables_map& vm, const std::string& bag_filename, const std::string& camera_info_topic, const std::string& image_topic);

  std::pair<cv::Mat, Frame::ConstPtr> get_image_and_points(
    const boost::program_options::variables_map& vm,
    const std::string& bag_filename,
    const std::string& image_topic,
    const std::string& points_topic,
    const std::string& intensity_channel,
    const int num_threads);
};

}  // namespace vlcal
