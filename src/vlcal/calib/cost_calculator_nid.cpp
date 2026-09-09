#include <vlcal/calib/cost_calculator_nid.hpp>
#include <algorithm>

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
  const double invalid_cost = std::numeric_limits<double>::infinity();
  if (!proj || !data || !data->points || !data->points->points || !data->points->intensities || params.bins < 2 || data->image.empty() || data->image.type() != CV_8UC1) {
    return invalid_cost;
  }
  const auto& image = data->image;
  const auto& points = data->points;

  Eigen::MatrixXi hist = Eigen::MatrixXi::Zero(params.bins, params.bins);
  Eigen::VectorXi hist_image = Eigen::VectorXi::Zero(params.bins);
  Eigen::VectorXi hist_points = Eigen::VectorXi::Zero(params.bins);

  for (int i = 0; i < points->size(); i++) {
    const Eigen::Vector4d pt_camera = T_camera_lidar * points->points[i];
    if (!proj->is_valid(pt_camera.head<3>()) || pt_camera.head<3>().normalized().z() < std::cos(max_fov) - 1e-12 || !std::isfinite(points->intensities[i])) {
      // Out of FoV
      continue;
    }

    const Eigen::Vector2d projected = proj->project(pt_camera.head<3>());
    if (!proj->is_pixel_valid(projected, image.cols, image.rows)) {
      // Out of Image
      continue;
    }
    const Eigen::Array2i pt_2d = projected.cast<int>();

    const double pixel = image.at<std::uint8_t>(pt_2d.y(), pt_2d.x()) / 255.0;
    const double lidar_intensity = points->intensities[i];

    const int image_bin = std::max<int>(0, std::min<int>(params.bins - 1, pixel * params.bins));
    const int lidar_bin = std::min<int>(params.bins - 1, std::clamp(lidar_intensity, 0.0, 1.0) * params.bins);

    hist(image_bin, lidar_bin)++;
    hist_image[image_bin]++;
    hist_points[lidar_bin]++;
  }

  const int sum = hist_image.sum();
  if (sum <= 0) {
    return invalid_cost;
  }
  const Eigen::MatrixXd hist_rs = hist.cast<double>() / sum;
  const Eigen::VectorXd hist_r = hist_image.cast<double>() / sum;
  const Eigen::VectorXd hist_s = hist_points.cast<double>() / sum;

  const double Hr = -(hist_r.array() * (hist_r.array() + 1e-6).log()).sum();
  const double Hs = -(hist_s.array() * (hist_s.array() + 1e-6).log()).sum();
  const double Hrs = -(hist_rs.array() * (hist_rs.array() + 1e-6).log()).sum();
  if (!std::isfinite(Hr) || !std::isfinite(Hs) || !std::isfinite(Hrs) || Hr <= 1e-8 || Hs <= 1e-8 || Hrs <= 1e-8) {
    return invalid_cost;
  }

  const double MI = Hr + Hs - Hrs;
  const double NID = (Hrs - MI) / Hrs;

  return std::isfinite(NID) ? NID : invalid_cost;
}

}  // namespace vlcal
