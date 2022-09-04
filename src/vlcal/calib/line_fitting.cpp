#include <vlcal/calib/line_fitting.hpp>

#include <Eigen/Core>
#include <Eigen/Eigen>

namespace vlcal {

// Through some tests, it seems LSQ estimation is enough and robust estimation is not necessary
std::pair<Eigen::Vector2d, Eigen::Vector2d> fit_line(const std::vector<Eigen::Vector2i>& points) {
  Eigen::Vector2d sum_pts = Eigen::Vector2d::Zero();
  Eigen::Matrix2d sum_cross = Eigen::Matrix2d::Zero();

  for (const auto& pt_i : points) {
    const Eigen::Vector2d pt = pt_i.cast<double>();
    sum_pts += pt;
    sum_cross += pt * pt.transpose();
  }

  const Eigen::Vector2d mean = sum_pts / points.size();
  const Eigen::Matrix2d cov = (sum_cross - mean * sum_pts.transpose()) / points.size();

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig;
  eig.computeDirect(cov);

  const Eigen::Vector2d normal = eig.eigenvectors().col(0).normalized();

  return std::make_pair(mean, normal);
}

}  // namespace vlcal
