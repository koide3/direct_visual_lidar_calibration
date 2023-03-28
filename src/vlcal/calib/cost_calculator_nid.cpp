#include <vlcal/calib/cost_calculator_nid.hpp>

#include <vlcal/common/estimate_fov.hpp>

namespace vlcal {

NIDCostParams::NIDCostParams() {
  bins = 16;
}

NIDCostParams::~NIDCostParams() {}

CostCalculatorNID::CostCalculatorNID(const camera::GenericCameraBase::ConstPtr& proj, const VisualLiDARData::ConstPtr& data, const NIDCostParams& params)
: params(params),
  proj(proj),
  data(data),
  max_fov(estimate_camera_fov(proj, {data->image.cols, data->image.rows})) {}

CostCalculatorNID::~CostCalculatorNID() {}

double CostCalculatorNID::calculate(const Eigen::Isometry3d& T_camera_lidar) {
  const auto& image = data->image;
  const auto& points = data->points;
  const Eigen::Array2i image_size(image.cols, image.rows);

  Eigen::MatrixXi hist = Eigen::MatrixXi::Zero(params.bins, params.bins);
  Eigen::VectorXi hist_image = Eigen::VectorXi::Zero(params.bins);
  Eigen::VectorXi hist_points = Eigen::VectorXi::Zero(params.bins);

  for (int i = 0; i < points->size(); i++) {
    const Eigen::Vector4d pt_camera = T_camera_lidar * points->points[i];
    if (pt_camera.head<3>().normalized().z() < std::cos(max_fov)) {
      // Out of FoV
      continue;
    }

    const Eigen::Array2i pt_2d = proj->project(pt_camera.head<3>()).cast<int>();
    if ((pt_2d < Eigen::Array2i::Zero()).any() || (pt_2d >= image_size).any()) {
      // Out of Image
      continue;
    }

    const double pixel = image.at<std::uint8_t>(pt_2d.y(), pt_2d.x()) / 255.0;
    const double lidar_intensity = points->intensities[i];

    const int image_bin = std::max<int>(0, std::min<int>(params.bins - 1, pixel * params.bins));
    const int lidar_bin = std::max<int>(0, std::min<int>(params.bins - 1, lidar_intensity * params.bins));

    hist(image_bin, lidar_bin)++;
    hist_image[image_bin]++;
    hist_points[lidar_bin]++;
  }

  const int sum = hist_image.sum();
  const Eigen::MatrixXd hist_rs = hist.cast<double>() / sum;
  const Eigen::VectorXd hist_r = hist_image.cast<double>() / sum;
  const Eigen::VectorXd hist_s = hist_points.cast<double>() / sum;

  const double Hr = -(hist_r.array() * (hist_r.array() + 1e-6).log()).sum();
  const double Hs = -(hist_s.array() * (hist_s.array() + 1e-6).log()).sum();
  const double Hrs = -(hist_rs.array() * (hist_rs.array() + 1e-6).log()).sum();

  const double MI = Hr + Hs - Hrs;
  const double NID = (Hrs - MI) / Hrs;

  return NID;
}

}  // namespace vlcal