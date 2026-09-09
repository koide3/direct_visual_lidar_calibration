#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

#include <ceres/jet.h>
#include <sophus/se3.hpp>
#include <camera/create_camera.hpp>
#include <vlcal/calib/cost_calculator_nid.hpp>
#include <vlcal/calib/view_culling.hpp>
#include <vlcal/common/estimate_fov.hpp>
#include <vlcal/costs/nid_cost.hpp>

namespace {

void require(bool condition, const char* message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

Eigen::Vector3d ray(double theta_deg) {
  const double theta = theta_deg * M_PI / 180.0;
  return {std::sin(theta), 0.0, std::cos(theta)};
}

vlcal::FrameCPU::Ptr make_points(const std::vector<Eigen::Vector3d>& xyz) {
  auto points = std::make_shared<vlcal::FrameCPU>(xyz);
  std::vector<double> intensities(xyz.size());
  for (size_t i = 0; i < xyz.size(); ++i) {
    intensities[i] = (i + 0.5) / xyz.size();
  }
  points->add_intensities(intensities);
  return points;
}

void test_projection_and_geometry() {
  const std::vector<double> intrinsics{3101.3277108433736, 3099.672289156627, 1917.6714285714286, 1932.75, 2.0};
  const std::vector<double> distortion{0.19911757, 2.07707953, 0.00030418, -0.0027582, -3.2787478};
  const auto mei = camera::create_camera("mei", intrinsics, distortion);
  const auto legacy = camera::create_camera("omnidir", intrinsics, distortion);
  require(mei && legacy, "camera factory failed");
  const double delta = mei->project(Eigen::Vector3d::UnitX()).x() - legacy->project(Eigen::Vector3d::UnitX()).x();
  // At theta=90 degrees and xi=2: x_u=1/2, delta_u=fx*k3*x_u^7.
  require(std::abs(delta - intrinsics[0] * distortion[4] / 128.0) < 1e-9, "MEI k3 contribution is incorrect");
  require((mei->project(ray(45.0) * 1e200) - mei->project(ray(45.0))).norm() < 1e-9, "finite large ray overflowed during normalization");
  require(mei->is_valid(ray(95.0)), "valid 95-degree ray with negative z was discarded");
  require(!mei->is_valid(ray(95.01)), "configured half-angle was ignored");
  require(!mei->is_valid(Eigen::Vector3d::Zero()), "zero ray accepted");
  require(!mei->is_valid({std::numeric_limits<double>::infinity(), 0.0, 1.0}), "infinite ray accepted");
  require(!mei->is_valid({std::numeric_limits<double>::quiet_NaN(), 0.0, 1.0}), "NaN ray accepted");
  require(std::abs(vlcal::estimate_camera_fov(mei, {3840, 3840}) - 95.0 * M_PI / 180.0) < 1e-12, "explicit FoV not used");

  camera::CameraValidity wide;
  wide.max_theta_deg = 179.0;
  const auto wide_mei = camera::create_camera("mei", intrinsics, distortion, wide);
  require(wide_mei->is_valid(ray(100.0)), "valid physical branch rejected");
  require(!wide_mei->is_valid(ray(120.0)), "branch singularity accepted");
  require(!wide_mei->is_valid(ray(130.0)), "folded MEI branch accepted because its denominator is positive");

  const Eigen::Matrix3d back_rotation = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()).toRotationMatrix();
  require(mei->is_valid(Eigen::Vector3d::UnitZ()) && !mei->is_valid(back_rotation * Eigen::Vector3d::UnitZ()), "back-to-back front visibility is incorrect");
  require(!mei->is_valid(-Eigen::Vector3d::UnitZ()) && mei->is_valid(back_rotation * -Eigen::Vector3d::UnitZ()), "back-to-back rear visibility is incorrect");

  const auto points = make_points({ray(0.0) * 5.0, ray(95.0) * 5.0, ray(130.0) * 5.0, Eigen::Vector3d::Zero()});
  vlcal::ViewCullingParams params;
  params.enable_depth_buffer_culling = false;
  vlcal::ViewCulling culling(mei, {3840, 3840}, params);
  require(culling.cull(points, Eigen::Isometry3d::Identity())->size() == 2, "view culling failed the MEI physical/FoV checks");

  using Jet = ceres::Jet<double, 7>;
  Eigen::Matrix<Jet, 3, 1> differentiable;
  differentiable << Jet(0.3), Jet(0.2), Jet(1.0);
  differentiable.x().v[0] = 1.0;
  const auto projected = (*mei)(differentiable);
  require(std::isfinite(projected.x().a) && projected.x().v.allFinite() && std::abs(projected.x().v[0]) > 1.0, "MEI autodiff projection failed");
}

void test_mask_footprint() {
  camera::CameraValidity validity;
  validity.image_mask = cv::Mat(16, 16, CV_8UC1, cv::Scalar(255));
  validity.image_mask.at<unsigned char>(10, 10) = 0;
  const auto camera = camera::create_camera("mei", {10, 10, 8, 8, 2}, {0, 0, 0, 0, 0}, validity);
  require(camera->is_pixel_valid({8.2, 8.2}, 16, 16), "valid center pixel rejected");
  require(!camera->is_pixel_valid({8.2, 8.2}, 16, 16, 1, 2), "4x4 footprint missed an invalid corner");
  require(camera->is_pixel_valid({4.2, 4.2}, 16, 16, 1, 2), "valid full footprint rejected");
  require(!camera->is_pixel_valid({0.5, 8.2}, 16, 16, 1, 2), "out-of-image spline footprint accepted");
  require(!camera->is_pixel_valid({std::numeric_limits<double>::quiet_NaN(), 8.2}, 16, 16), "NaN pixel accepted");
  require(!camera->is_pixel_valid({4.2, 4.2}, 32, 16), "mismatched mask dimensions accepted");
  validity.image_mask.setTo(255);
  require(!camera->is_pixel_valid({8.2, 8.2}, 16, 16, 1, 2), "camera did not own a stable mask copy");
}

void test_nid_failures_and_per_pose_validity() {
  const std::vector<double> intrinsics{30, 30, 16, 16, 2};
  const std::vector<double> distortion(5, 0.0);
  const auto camera = camera::create_camera("mei", intrinsics, distortion);
  std::vector<Eigen::Vector3d> xyz;
  for (int y = -3; y <= 3; ++y) {
    for (int x = -3; x <= 3; ++x) {
      xyz.emplace_back(x * 0.25, y * 0.25, 1.0);
    }
  }
  const auto points = make_points(xyz);
  cv::Mat image(32, 32, CV_8UC1);
  for (int y = 0; y < image.rows; ++y) {
    for (int x = 0; x < image.cols; ++x) {
      image.at<unsigned char>(y, x) = (3 * x + 5 * y) % 256;
    }
  }
  cv::Mat normalized;
  image.convertTo(normalized, CV_64FC1, 1.0 / 255.0);
  vlcal::NIDCost nid(camera, normalized, points);
  const Sophus::SE3d identity;
  double cost = 0.0;
  require(nid(identity.data(), &cost) && std::isfinite(cost), "valid textured NID fixture failed");
  auto data = std::make_shared<vlcal::VisualLiDARData>(image, points);
  vlcal::CostCalculatorNID discrete_nid(camera, data);
  require(std::isfinite(discrete_nid.calculate(Eigen::Isometry3d::Identity())), "valid discrete NID fixture failed");

  Eigen::Isometry3d flipped = Eigen::Isometry3d::Identity();
  flipped.linear() = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()).toRotationMatrix();
  const Sophus::SE3d backwards(flipped.matrix());
  require(!nid(backwards.data(), &cost), "NID did not recheck the physical branch after pose changed");
  require(std::isinf(discrete_nid.calculate(flipped)), "discrete NID did not reject zero valid samples");

  const auto empty = make_points({});
  vlcal::NIDCost empty_nid(camera, normalized, empty);
  require(!empty_nid(identity.data(), &cost), "empty NID cloud accepted");
  const cv::Mat flat_image(32, 32, CV_64FC1, cv::Scalar(0.5));
  vlcal::NIDCost flat_nid(camera, flat_image, points);
  require(!flat_nid(identity.data(), &cost), "zero image entropy accepted");

  camera::CameraValidity masked;
  masked.image_mask = cv::Mat(32, 32, CV_8UC1, cv::Scalar(0));
  const auto masked_camera = camera::create_camera("mei", intrinsics, distortion, masked);
  vlcal::NIDCost masked_nid(masked_camera, normalized, points);
  require(!masked_nid(identity.data(), &cost), "NID accepted a completely masked image");
  vlcal::CostCalculatorNID masked_discrete(masked_camera, data);
  require(std::isinf(masked_discrete.calculate(Eigen::Isometry3d::Identity())), "discrete NID ignored mask");
}

}  // namespace

int main() {
  try {
    test_projection_and_geometry();
    test_mask_footprint();
    test_nid_failures_and_per_pose_validity();
    std::cout << "MEI projection, branch, mask, and NID validation tests passed" << std::endl;
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "MEI validation test failed: " << error.what() << std::endl;
    return 1;
  }
}
