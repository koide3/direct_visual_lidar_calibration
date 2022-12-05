#pragma once

#include <algorithm>
#include <sophus/se3.hpp>
#include <opencv2/core.hpp>
#include <vlcal/common/frame.hpp>
#include <camera/generic_camera_base.hpp>

namespace vlcal {

template <typename T>
double get_real(const T& x) {
  return x.a;
}

template <>
double get_real(const double& x) {
  return x;
}

class NIDCost {
public:
  NIDCost(const camera::GenericCameraBase::ConstPtr& proj, const cv::Mat& normalized_image, const Frame::ConstPtr& points, const int bins = 16)
  : proj(proj),
    normalized_image(normalized_image.clone()),
    points(points),
    bins(bins) {
    //
    spline_coeffs.row(0) << 1.0, -3.0, 3.0, -1.0;
    spline_coeffs.row(1) << 4.0, 0.0, -6.0, 3.0;
    spline_coeffs.row(2) << 1.0, 3.0, 3.0, -3.0;
    spline_coeffs.row(3) << 0.0, 0.0, 0.0, 1.0;
    spline_coeffs /= 6.0;
  }

  template <typename T>
  bool operator()(const T* T_camera_lidar_params, T* residual) const {
    const Eigen::Map<Sophus::SE3<T> const> T_camera_lidar(T_camera_lidar_params);

    Eigen::Matrix<T, -1, -1> hist = Eigen::Matrix<T, -1, -1>::Zero(bins, bins);

    Eigen::Matrix<T, -1, 1> hist_image = Eigen::Matrix<T, -1, 1>::Zero(bins);
    Eigen::VectorXd hist_points = Eigen::VectorXd::Zero(bins);

    int num_outliers = 0;
    for (int i = 0; i < points->size(); i++) {
      const Eigen::Matrix<T, 3, 1> pt_camera = T_camera_lidar * points->points[i].head<3>();
      const double intensity = points->intensities[i];
      const int bin_points = std::max<int>(0, std::min<int>(bins - 1, intensity * bins));

      const Eigen::Matrix<T, 2, 1> projected = (*proj)(pt_camera);
      const Eigen::Vector2i knot_i(std::floor(get_real(projected[0])), std::floor(get_real(projected[1])));
      const Eigen::Matrix<T, 2, 1> s = projected - knot_i.cast<double>();

      if ((knot_i.array() < Eigen::Array2i(0, 0)).any() || (knot_i.array() >= Eigen::Array2i(normalized_image.cols, normalized_image.rows)).any()) {
        num_outliers++;
        continue;
      }

      hist_points[bin_points]++;

      Eigen::Matrix<T, 4, 2> se;
      se.row(0).setOnes();
      se.row(1) = s.transpose();
      se.row(2) = s.array().square().transpose();
      se.row(3) = (s.array().square() * s.array()).transpose();

      const Eigen::Matrix<T, 4, 2> beta = spline_coeffs * se;

      Eigen::Array4i knots_x(knot_i.x() - 1, knot_i.x(), knot_i.x() + 1, knot_i.x() + 2);
      Eigen::Array4i knots_y(knot_i.y() - 1, knot_i.y(), knot_i.y() + 1, knot_i.y() + 2);
      knots_x = knots_x.max(0).min(normalized_image.cols - 1);
      knots_y = knots_y.max(0).min(normalized_image.rows - 1);

      for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
          const T w = beta(i, 0) * beta(j, 1);
          const double pix = normalized_image.at<double>(knots_y[j], knots_x[i]);
          const int bin_image = std::min<int>(pix * bins, bins - 1);
          hist(bin_image, bin_points) += w;
          hist_image[bin_image] += w;
        }
      }
    }

    const double sum = hist_points.sum();

    hist_image = hist_image / sum;
    hist_points = hist_points / sum;
    hist = hist / sum;

    const T H_image = -(hist_image.array() * (hist_image.array() + 1e-6).log()).sum();
    const double H_points = -(hist_points.array() * (hist_points.array() + 1e-6).log()).sum();
    const T H_image_points = -(hist.array() * (hist.array() + 1e-6).log()).sum();
    const T MI = H_image + H_points - H_image_points;
    const T NID = (H_image_points - MI) / H_image_points;

    if (!std::isfinite(get_real(NID))) {
      std::cout << get_real(H_image_points) << " " << get_real(MI) << " " << get_real(NID) << std::endl;

      return false;
    }

    residual[0] = NID;

    return true;
  }

private:
  const camera::GenericCameraBase::ConstPtr proj;
  const cv::Mat normalized_image;
  const Frame::ConstPtr points;

  const int bins;
  Eigen::Matrix<double, 4, 4> spline_coeffs;
};
}