#include <vlcal/common/visual_lidar_data.hpp>

#include <opencv2/opencv.hpp>
#include <vlcal/common/console_colors.hpp>

#include <glk/io/ply_io.hpp>

namespace vlcal {

VisualLiDARData::VisualLiDARData(const std::string& data_path, const std::string& bag_name) {
  std::cout << "loading " << data_path + "/" + bag_name + ".(png|ply)" << std::endl;

  image = cv::imread(data_path + "/" + bag_name + ".png", 0);
  if (!image.data) {
    std::cerr << vlcal::console::bold_red << "warning: failed to load " << data_path + "/" + bag_name + ".png" << vlcal::console::reset << std::endl;
    abort();
  }

  auto ply = glk::load_ply(data_path + "/" + bag_name + ".ply");
  if (!ply) {
    std::cerr << vlcal::console::bold_red << "warning: failed to load " << data_path + "/" + bag_name + ".ply" << vlcal::console::reset << std::endl;
    abort();
  }

  points = std::make_shared<FrameCPU>(ply->vertices);
  points->add_intensities(ply->intensities);
}

VisualLiDARData::~VisualLiDARData() {}

}