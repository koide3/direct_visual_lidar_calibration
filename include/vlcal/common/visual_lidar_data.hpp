#include <memory>
#include <iostream>
#include <opencv2/core.hpp>
#include <gtsam_ext/types/frame_cpu.hpp>

namespace vlcal {

struct VisualLiDARData {
public:
  using Ptr = std::shared_ptr<VisualLiDARData>;
  using ConstPtr = std::shared_ptr<const VisualLiDARData>;

  VisualLiDARData(const std::string& data_path, const std::string& bag_name);
  ~VisualLiDARData();

public:
  cv::Mat image;
  gtsam_ext::FrameCPU::Ptr points;
};

}  // namespace vlcal
