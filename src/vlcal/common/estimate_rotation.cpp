#include <vlcal/common/estimate_rotation.hpp>

#include <random>
#include <vlcal/common/estimate_fov.hpp>

namespace vlcal {

std::pair<int, Eigen::Matrix3d> estimate_rotation_ransac(
  const camera::GenericCameraBase::ConstPtr& proj,
  const std::vector<std::pair<Eigen::Vector2d, Eigen::Vector4d>>& correspondences,
  const double projection_error_thresh,
  const int iterations) {
  // Compute bearing vectors
  std::vector<Eigen::Vector4d> directions_camera(correspondences.size());
  std::vector<Eigen::Vector4d> directions_lidar(correspondences.size());
  for (int i = 0; i < correspondences.size(); i++) {
    directions_camera[i] << estimate_direction(proj, correspondences[i].first), 0.0;
    directions_lidar[i] << correspondences[i].second.head<3>().normalized(), 0.0;
  }

  // LSQ rotation estimation
  // https://web.stanford.edu/class/cs273/refs/umeyama.pdf
  const auto find_rotation = [&](const int index0, const int index1) {
    const auto& d_c0 = directions_camera[index0];
    const auto& d_c1 = directions_camera[index1];
    const auto& d_l0 = directions_lidar[index0];
    const auto& d_l1 = directions_lidar[index1];

    const Eigen::Matrix<double, 3, 2> A = (Eigen::Matrix<double, 3, 2>() << d_c0.head<3>(), d_c1.head<3>()).finished();
    const Eigen::Matrix<double, 3, 2> B = (Eigen::Matrix<double, 3, 2>() << d_l0.head<3>(), d_l1.head<3>()).finished();
    const Eigen::Matrix3d AB = A * B.transpose();

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(AB, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const Eigen::Matrix3d U = svd.matrixU();
    const Eigen::Matrix3d V = svd.matrixV();
    const Eigen::Matrix3d D = svd.singularValues().asDiagonal();
    Eigen::Matrix3d S = Eigen::Matrix3d::Identity();

    double det = U.determinant() * V.determinant();
    if (det < 0.0) {
      S(2, 2) = -1.0;
    }

    const Eigen::Matrix3d R_camera_lidar = U * S * V.transpose();
    return R_camera_lidar;
  };

  const double error_thresh_sq = projection_error_thresh * projection_error_thresh;

  int best_inliers = 0;
  Eigen::Matrix4d best_R_camera_lidar;

  std::mt19937 mt;
  std::vector<std::mt19937> mts(omp_get_max_threads());
  for (int i = 0; i < mts.size(); i++) {
    mts[i] = std::mt19937(mt() + 8192 * i);
  }

#pragma omp parallel for
  for (int i = 0; i < iterations; i++) {
    const int thread_id = omp_get_thread_num();
    std::uniform_int_distribution<> udist(0, correspondences.size() - 1);
    const int index0 = udist(mts[thread_id]);
    const int index1 = udist(mts[thread_id]);

    if (index0 == index1) {
      continue;
    }

    Eigen::Matrix4d R_camera_lidar = Eigen::Matrix4d::Zero();
    R_camera_lidar.topLeftCorner<3, 3>() = find_rotation(index0, index1);

    int inliers = 0;
    for (int j = 0; j < correspondences.size(); j++) {
      const Eigen::Vector4d direction_cam = R_camera_lidar * directions_lidar[j];
      const Eigen::Vector2d pt_2d = proj->project(direction_cam.head<3>());
      if ((correspondences[j].first - pt_2d).squaredNorm() < error_thresh_sq) {
        inliers++;
      }
    }

#pragma omp critical
    if (inliers > best_inliers) {
      best_inliers = inliers;
      best_R_camera_lidar = R_camera_lidar;
    }
  }

  return std::make_pair(best_inliers, best_R_camera_lidar.topLeftCorner<3, 3>(0, 0));
}

}  // namespace vlcal
